#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>

#define SLAVE_ADDRESS 0x01

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(nullptr)
    , responseTimer(nullptr)
    , currentRequest(REQ_NONE)
{
    ui->setupUi(this);

    //initSerialPort();

    responseTimer = new QTimer(this);
    responseTimer->setSingleShot(true);
    responseTimer->setInterval(1000);
    connect(responseTimer, &QTimer::timeout, this, &MainWindow::handleTimeout);

    ui->lblOutput->setText("Hazır");
}

MainWindow::~MainWindow()
{
    if (serialPort && serialPort->isOpen()) { // serialPort null değilse (null olursa ve  serialPort && okontrolu olmazsa program çöker) ve açık konumdaysa koşuluna bakıyor
        serialPort->close();
    }
    delete ui;
}

void MainWindow::initSerialPort()
{
    serialPort = new QSerialPort(this);

    serialPort->setPortName(ui->cmbPortList->currentText());
    serialPort->setBaudRate(ui->cmbBaudRate->currentText().toInt());
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop); // Stop bitle flowcontrole gerek yok bunları kaldırabilirsin
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort->open(QIODevice::ReadWrite)) {
        ui->lblOutput->setText("Port opened successfully!");
        ui->btnConnection->setText("Disconnect");
    } else {
        ui->lblOutput->setText("Failed to open port!");
    }

    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

}

quint16 MainWindow::calculateCRC16(const QByteArray &data)
{
    quint16 crc = 0xFFFF;

    for (int i = 0; i < data.size(); i++) {
        crc ^= static_cast<quint8>(data[i]);

        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

float MainWindow::registersToFloat(quint16 high, quint16 low)
{
    // STM32'deki Float_To_Registers() ile aynı mantık
    // high = MSB (üst 16 bit), low = LSB (alt 16 bit)
    union {
        float f;
        quint32 u;
    } data;

    data.u = (static_cast<quint32>(high) << 16) | static_cast<quint32>(low);
    return data.f;
}

void MainWindow::sendModbusRequest(quint8 slaveAddr, quint8 functionCode,
                                   quint16 startAddr, quint16 numRegisters)
{
    if (!serialPort || !serialPort->isOpen()) {
        QMessageBox::warning(this, "Uyarı", "Seri port açık değil!");
        return;
    }

    QByteArray request; // Bellekte (stackte çünkü başka fonk.larla buray erişmeyeceğimiz için derleyici işini bitirince bu nesneyi otomatik silsin istiyorum) bir QByteArray nesnesi oluşturuluyor
    request.append(static_cast<char>(slaveAddr));
    request.append(static_cast<char>(functionCode));
    request.append(static_cast<char>((startAddr >> 8) & 0xFF));
    request.append(static_cast<char>(startAddr & 0xFF));
    request.append(static_cast<char>((numRegisters >> 8) & 0xFF)); // ilk yüksek 8 biti alıyor
    request.append(static_cast<char>(numRegisters & 0xFF)); // düşük 8 biti alıyor

    quint16 crc = calculateCRC16(request);
    request.append(static_cast<char>(crc & 0xFF));
    request.append(static_cast<char>((crc >> 8) & 0xFF));

    responseBuffer.clear();
    serialPort->clear();

    serialPort->write(request);
    serialPort->flush();

    responseTimer->start();

    qDebug() << "Modbus Request gönderildi:" << request.toHex(' ');
}

void MainWindow::readSerialData()
{
    responseBuffer.append(serialPort->readAll());


    if (responseBuffer.size() >= 7) {
        responseTimer->stop();
        parseModbusResponse(responseBuffer);
        responseBuffer.clear();
    }
}

void MainWindow::parseModbusResponse(const QByteArray &response)
{
    qDebug() << "Modbus Response alındı:" << response.toHex(' ');

    if (response.size() < 7) {
        ui->lblOutput->setText("Hata: Yanıt çok kısa!");
        return;
    }

    // CRC kontrolü
    QByteArray dataWithoutCRC = response.left(response.size() - 2);
    quint16 receivedCRC = (static_cast<quint8>(response[response.size() - 1]) << 8) |
                          static_cast<quint8>(response[response.size() - 2]);
    quint16 calculatedCRC = calculateCRC16(dataWithoutCRC);

    if (receivedCRC != calculatedCRC) {
        ui->lblOutput->setText(QString("Hata: CRC uyuşmuyor! (Alınan: %1, Hesaplanan: %2)")
                                   .arg(receivedCRC, 4, 16, QChar('0'))
                                   .arg(calculatedCRC, 4, 16, QChar('0')));
        return;
    }

    quint8 slaveId = static_cast<quint8>(response[0]);
    quint8 functionCode = static_cast<quint8>(response[1]);

    // Slave ID kontrolü
    if (slaveId != SLAVE_ADDRESS) {
        ui->lblOutput->setText("Hata: Geçersiz Slave ID!");
        return;
    }

    // Exception kontrolü
    if (functionCode & 0x80) {
        quint8 exceptionCode = static_cast<quint8>(response[2]);
        ui->lblOutput->setText(QString("Modbus Exception: Code %1").arg(exceptionCode));
        return;
    }

    if (functionCode != 0x03) {
        ui->lblOutput->setText("Hata: Geçersiz fonksiyon kodu!");
        return;
    }

    quint8 byteCount = static_cast<quint8>(response[2]);

    // Register'ları oku
    QVector<quint16> registers;
    for (int i = 0; i < byteCount; i += 2) { // her register 2 byte olduğu için döngü 2’şer adımlarla ilerler.
        quint16 value = (static_cast<quint8>(response[3 + i]) << 8) |
                        static_cast<quint8>(response[3 + i + 1]); // Veri kısmı 3.indexten itibaren başladığı için ve amacımız 8 (Data_1_High) + 8 (Data_1_Low) biti birleştirip 16 bitlik veri oluşturmak olduğu için response[3 + i] ve response[3 + i + 1] kullanılıyor
        registers.append(value);
    }

    qDebug() << "Registers:" << registers;

    // İsteğe göre işle
    switch (currentRequest) {
    case REQ_TEMP:
        if (registers.size() >= 2) {
            float temp = registersToFloat(registers[0], registers[1]);
            ui->lblOutput->setText("Sıcaklık: " + formatTemperature(temp));
            qDebug() << "Temperature:" << temp;
        } else {
            ui->lblOutput->setText("Hata: Yetersiz veri!");
        }
        break;

    case REQ_VREF:
        if (registers.size() >= 2) {
            float vref = registersToFloat(registers[0], registers[1]);
            ui->lblOutput->setText("Vref: " + formatVoltage(vref));
            qDebug() << "Vref:" << vref;
        } else {
            ui->lblOutput->setText("Hata: Yetersiz veri!");
        }
        break;

    case REQ_ALL:
        if (registers.size() >= 4) {
            float temp = registersToFloat(registers[0], registers[1]);
            float vref = registersToFloat(registers[2], registers[3]);
            quint16 buttonState = registers[4];  // 5. register buton durumu
            QString allData = "Sıcaklık: " + formatTemperature(temp) + "\n";
            allData += "Vref: " + formatVoltage(vref) + "\n";
            allData += "Buton: " + formatButtonState(buttonState);  // Buton durumunu ekle
            ui->lblOutput->setText(allData);

            qDebug() << "Temperature:" << temp << "Vref:" << vref << "Buton Durumu:" << buttonState;
        } else {
            ui->lblOutput->setText("Hata: Yetersiz veri!");
        }
        break;

    case REQ_BUTTON_STATE:
        if (registers.size() >= 1) { // Gelen register sayısının en az 1 olup olmadığını kontrol eder
            quint16 buttonState = registers[0]; // İlk (ve tek) register'ı okur - bu butonun durumunu içerir (0 veya 1)
            ui->lblOutput->setText("Buton Durumu: " + formatButtonState(buttonState));
            qDebug() << "Button State:" << buttonState;
        } else {
            ui->lblOutput->setText("Hata: Yetersiz veri!");
        }
        break;

    default:
        break;
    }

    currentRequest = REQ_NONE;
}

QString MainWindow::formatVoltage(float voltage)
{
    return QString::number(voltage, 'f', 3) + " V";
}

QString MainWindow::formatTemperature(float temp)
{
    return QString::number(temp, 'f', 1) + " °C";
}

QString MainWindow::formatButtonState(quint16 rawValue)
{
    return (rawValue == 0) ? "Basılı Değil" : "Basılı";
}

void MainWindow::handleTimeout()
{
    ui->lblOutput->setText("Hata: Timeout!");
    currentRequest = REQ_NONE;
}

void MainWindow::on_btnTemp_clicked()
{
    currentRequest = REQ_TEMP;
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_TEMP_HIGH, 2);
}

void MainWindow::on_btnVref_clicked()
{
    currentRequest = REQ_VREF;
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_VREF_HIGH, 2);
}

void MainWindow::on_btnButtonState_clicked()
{
    currentRequest = REQ_BUTTON_STATE;
    sendModbusRequest(SLAVE_ADDRESS, 0x03,REG_BUTTON_STATE,1);
}

void MainWindow::on_btnAll_clicked()
{
    currentRequest = REQ_ALL; // Qt kendisi arka planda hallediyor bu kısmı
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_ALL_START, REG_ALL_COUNT);
}

void MainWindow::on_btnConnect_clicked()
{
    if (serialPort && serialPort->isOpen()) {
        ui->lblOutput->setText("Zaten bağlı.");
        return;
    }

    // Port ismini GUI'deki comboBox'tan al
    QString selectedPort = ui->cmbPortList->currentText();
    serialPort->setPortName(selectedPort);

    applySerialPortSettings(); // Baudrate, databits vs ayarlarını uygula

    if (!serialPort->open(QIODevice::ReadWrite)) {
        QMessageBox::critical(this, "Bağlantı Hatası", "Port açılamadı: " + serialPort->errorString());
        return;
    }

    ui->lblOutput->setText("Bağlantı kuruldu: " + selectedPort);
}

void MainWindow::on_btnDisconnect_clicked()
{
    if (serialPort && serialPort->isOpen()) {
        serialPort->close();
        ui->lblOutput->setText("Bağlantı kesildi.");
    } else {
        ui->lblOutput->setText("Zaten bağlantı yok.");
    }
}
/*void MainWindow::on_btnConnection_clicked(){

}*/
void MainWindow::on_btnRefresh_clicked()
{
    ui->cmbPortList->clear();

    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports) {
        ui->cmbPortList->addItem(port.portName());
    }

    if (ports.isEmpty()) {
        ui->lblOutput->setText("Port bulunamadı.");
    } else {
        ui->lblOutput->setText("Portlar yenilendi.");
    }
}

void MainWindow::on_cmbPortList_currentIndexChanged(int index)
{
    if (index >= 0) {
        QString selected = ui->cmbPortList->itemText(index);
        updatePortInfo(selected); // Dilersen detaylarını göster
        ui->lblOutput->setText("Seçilen port: " + selected);
    }
}

void MainWindow::applySerialPortSettings()
{
    serialPort->setBaudRate(ui->cmbBaudRate->currentText().toInt());

    int dataBits = ui->cmbDataBits->currentText().toInt();
    switch (dataBits) {
    case 5: serialPort->setDataBits(QSerialPort::Data5); break;
    case 6: serialPort->setDataBits(QSerialPort::Data6); break;
    case 7: serialPort->setDataBits(QSerialPort::Data7); break;
    case 8: default: serialPort->setDataBits(QSerialPort::Data8); break;
    }

    QString parityStr = ui->cmbParity->currentText();
    if (parityStr == "0" || parityStr == "None")
        serialPort->setParity(QSerialPort::NoParity);
    else if (parityStr == "Even")
        serialPort->setParity(QSerialPort::EvenParity);
    else if (parityStr == "Odd")
        serialPort->setParity(QSerialPort::OddParity);

    // Stop bit sabit 1, GUI'den gelmiyorsa:
    serialPort->setStopBits(QSerialPort::OneStop);

    QString flowCtrl = ui->cmbFlowControl->currentText();
    if (flowCtrl == "None")
        serialPort->setFlowControl(QSerialPort::NoFlowControl);
    else if (flowCtrl == "RTS/CTS")
        serialPort->setFlowControl(QSerialPort::HardwareControl);
    else if (flowCtrl == "XON/XOFF")
        serialPort->setFlowControl(QSerialPort::SoftwareControl);
}

void MainWindow::updatePortInfo(const QString &portName)
{

    qDebug() << "Port seçildi:" << portName;

    // İstersen port hakkında detayları da gösterebilirsin:
    const QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports) {
        if (port.portName() == portName) {
            qDebug() << "Description:" << port.description();
            qDebug() << "Manufacturer:" << port.manufacturer();
            qDebug() << "Serial Number:" << port.serialNumber();
        }
    }
}


void MainWindow::on_btnConnection_clicked()
{
    if (ui->btnConnection->text() == "Disconnect") {
        // Port currently open → close it
        serialPort->close();
        ui->lblOutput->setText("Port closed!");
        ui->btnConnection->setText("Connect");
    } else {
        // Port closed → open it
        initSerialPort();
    }
}

