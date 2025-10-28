#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// STM32 kodundaki register adresleriyle aynı adresler kullanılır
#define REG_TEMP_HIGH     0x0000
#define REG_TEMP_LOW      0x0001
#define REG_VREF_HIGH     0x0002
#define REG_VREF_LOW      0x0003
#define REG_ALL_START     0x0000
#define REG_BUTTON_STATE  0x0004    //  buton register adresi
#define REG_ALL_COUNT     0x0005    //  5 register okunacak


#include <QMainWindow>       // Uygulamanın ana penceresini temsil eder
#include <QSerialPort>      // PC'nin seri portuyla (COM port) iletişim kurmanı sağlar write() ve readAll() burdan gelir
#include <QSerialPortInfo> // PC de takılı olan tüm serial portları listeler
#include <QTimer>         // İşlemleri zamanlamak için kullanılır
#include <QByteArray>    // Binary veri dizilerini depolamada kullanılır. UART tan gelen veya gönderilen byte verilerini saklar
#include <QVector>      // Registerları kodda dinamik olarak kullanır.

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_btnVref_clicked();
    void on_btnTemp_clicked();
    void on_btnButtonState_clicked();
    void on_btnAll_clicked();
    void readSerialData();
    void handleTimeout();

    void on_btnConnect_clicked();
    void on_btnDisconnect_clicked();
    void on_btnRefresh_clicked();
    void on_cmbPortList_currentIndexChanged(int index);

    void on_btnConnection_clicked();

private:
    Ui::MainWindow *ui; // Butonlara erişim sağlar
    QSerialPort *serialPort; // STM32 ile fiziksel iletişimi sağlar
    QTimer *responseTimer; // Eğer STM32’den belirli süre boyunca cevap gelmezse handleTimeout() fonksiyonunu tetikler
    QByteArray responseBuffer; // STM32’den gelen ham byte verisini geçici olarak tutar


    void loadAvailablePorts();
    void updatePortInfo(const QString &portName);
    void applySerialPortSettings();

    enum RequestType {
        REQ_NONE = 0,
        REQ_VREF = 1,
        REQ_TEMP = 2,
        REQ_BUTTON_STATE = 3,
        REQ_ALL = 4
    };
    RequestType currentRequest;

    void initSerialPort();
    void sendModbusRequest(quint8 slaveAddr, quint8 functionCode, quint16 startAddr, quint16 numRegisters);
    quint16 calculateCRC16(const QByteArray &data);
    void parseModbusResponse(const QByteArray &response);
    float registersToFloat(quint16 high, quint16 low);
    QString formatVoltage(float voltage);
    QString formatTemperature(float temp);
    QString formatButtonState(quint16 rawValue);
};
#endif // MAINWINDOW_H
