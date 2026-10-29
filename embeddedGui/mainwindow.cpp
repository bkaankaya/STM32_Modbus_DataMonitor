#include "mainwindow.h"
#include "modbusframe.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>
#include <cstring>
#include <cstdint>

#define SLAVE_ADDRESS 0x01

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPort(new QSerialPort(this))
    , responseTimer(nullptr)
    , currentRequest(REQ_NONE)
    , pendingRegisters(0)
{
    ui->setupUi(this);

    connect(serialPort, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    responseTimer = new QTimer(this);
    responseTimer->setSingleShot(true);
    responseTimer->setInterval(1000);
    connect(responseTimer, &QTimer::timeout, this, &MainWindow::handleTimeout);

    on_btnRefresh_clicked();
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
    if (serialPort->isOpen())
        return;

    const QString selectedPort = ui->cmbPortList->currentText();
    if (selectedPort.isEmpty()) {
        ui->lblOutput->setText("No serial port selected.");
        return;
    }

    serialPort->setPortName(selectedPort);
    applySerialPortSettings();

    if (serialPort->open(QIODevice::ReadWrite)) {
        responseBuffer.clear();
        pendingRegisters = 0;
        currentRequest = REQ_NONE;
        ui->lblOutput->setText("Connected: " + selectedPort);
        ui->btnConnection->setText("Disconnect");
    } else {
        ui->lblOutput->setText("Cannot open port: " + serialPort->errorString());
        ui->btnConnection->setText("Connect");
    }
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
    const quint32 bits = (static_cast<quint32>(high) << 16) | static_cast<quint32>(low);
    float value;
    static_assert(sizeof(value) == sizeof(bits), "Expected a 32-bit float");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void MainWindow::sendModbusRequest(quint8 slaveAddr, quint8 functionCode,
                                   quint16 startAddr, quint16 numRegisters, RequestType requestType)
{
    if (!serialPort || !serialPort->isOpen()) {
        QMessageBox::warning(this, "Uyarı", "Seri port açık değil!");
        return;
    }

    if (responseTimer->isActive()) {
        ui->lblOutput->setText("Please wait for the previous request.");
        return;
    }
    if (functionCode != 0x03 || numRegisters == 0 || numRegisters > REG_ALL_COUNT) {
        ui->lblOutput->setText("Invalid register request.");
        return;
    }

    QByteArray request;
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
    serialPort->clear(QSerialPort::Input);
    if (serialPort->write(request) != request.size()) {
        ui->lblOutput->setText("Unable to queue serial request.");
        return;
    }

    currentRequest = requestType;
    pendingRegisters = numRegisters;
    responseTimer->start();

    qDebug() << "Modbus Request gönderildi:" << request.toHex(' ');
}

void MainWindow::readSerialData()
{
    const QByteArray incoming = serialPort->readAll();
    if (!responseTimer->isActive() || pendingRegisters == 0)
        return; // Ignore unsolicited or late bytes after a timeout.

    responseBuffer.append(incoming);
    const auto status = modbus::inspectResponse(
        reinterpret_cast<const std::uint8_t *>(responseBuffer.constData()),
        static_cast<std::size_t>(responseBuffer.size()), SLAVE_ADDRESS, pendingRegisters);

    if (status == modbus::FrameStatus::NeedMore)
        return;

    responseTimer->stop();
    if (status == modbus::FrameStatus::Invalid) {
        ui->lblOutput->setText("Invalid Modbus response (length, function, address or CRC).");
    } else {
        parseModbusResponse(responseBuffer);
    }
    responseBuffer.clear();
    pendingRegisters = 0;
    currentRequest = REQ_NONE;
}

void MainWindow::parseModbusResponse(const QByteArray &response)
{
    qDebug() << "Modbus Response alındı:" << response.toHex(' ');

    // Modbus exception responses are five bytes; normal responses are longer.
    if (response.size() < 5) {
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
    if (byteCount % 2 != 0 || response.size() != 5 + byteCount ||
        byteCount != pendingRegisters * 2) {
        ui->lblOutput->setText("Invalid register count in response.");
        return;
    }

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
            ui->lblOutput->setText("VDDA (estimated): " + formatVoltage(vref));
            qDebug() << "Vref:" << vref;
        } else {
            ui->lblOutput->setText("Hata: Yetersiz veri!");
        }
        break;

    case REQ_ALL:
        if (registers.size() >= 5) {
            float temp = registersToFloat(registers[0], registers[1]);
            float vref = registersToFloat(registers[2], registers[3]);
            quint16 buttonState = registers[4];  // 5. register buton durumu
            QString allData = "Sıcaklık: " + formatTemperature(temp) + "\n";
            allData += "VDDA (est.): " + formatVoltage(vref) + "\n";
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
    responseBuffer.clear();
    pendingRegisters = 0;
    currentRequest = REQ_NONE;
}

void MainWindow::on_btnTemp_clicked()
{
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_TEMP_HIGH, 2, REQ_TEMP);
}

void MainWindow::on_btnVref_clicked()
{
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_VREF_HIGH, 2, REQ_VREF);
}

void MainWindow::on_btnButtonState_clicked()
{
    sendModbusRequest(SLAVE_ADDRESS, 0x03,REG_BUTTON_STATE,1, REQ_BUTTON_STATE);
}

void MainWindow::on_btnAll_clicked()
{
    sendModbusRequest(SLAVE_ADDRESS, 0x03, REG_ALL_START, REG_ALL_COUNT, REQ_ALL);
}

void MainWindow::on_btnConnect_clicked()
{
    initSerialPort();
}

void MainWindow::on_btnDisconnect_clicked()
{
    responseTimer->stop();
    responseBuffer.clear();
    pendingRegisters = 0;
    currentRequest = REQ_NONE;
    if (serialPort->isOpen())
        serialPort->close();
    ui->btnConnection->setText("Connect");
    ui->lblOutput->setText("Disconnected.");
}

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
    if (serialPort->isOpen())
        on_btnDisconnect_clicked();
    else
        initSerialPort();
}
