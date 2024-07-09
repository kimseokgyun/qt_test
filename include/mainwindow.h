#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QLineEdit>

#include <iostream>
#include <stdio.h>
#include <fcntl.h>

#include <QFile>
#include <QXmlStreamWriter>
#include <QDateTime>
#include <QDomDocument>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


inline void SetLEColor(QLineEdit *le, QString color){
    le->setStyleSheet("QLineEdit{background-color:"+color+"}");
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    //About bms
    uint16_t bms_voltage;
    uint16_t bms_current;
    uint16_t bms_status;
    uint16_t bms_ttf;
    uint16_t bms_tte;
    uint8_t bms_soc;
    uint8_t bms_soh;
    uint16_t bms_rc;
    uint16_t bms_ae;
    uint16_t bms_temp;


    float BMS_VOLTAGE;
    float BMS_CURRENT;
    uint16_t BMS_STATUS;
    unsigned int BMS_TTF;
    unsigned int BMS_TTE;
    unsigned int BMS_SOC;
    unsigned int BMS_SOH;
    float BMS_RC;
    float BMS_AE;
    float BMS_TEMP;

    QString savefilename_;
    int can_status;

    void    *canHandler;
    void    *canHandler2;

    unsigned int    state_station;
    unsigned int    state_amr;

    unsigned int    station_limit_sensor;

    float           station_current_charger;
    float           station_current_contact;

    int             station_voltage_charger;
    int             station_voltage_contact;


    float           amr_current_contact;
    float           amr_current_battery;

    int             amr_voltage_contact;
    int             amr_voltage_battery;

    QDateTime startTime; // 타이머 시작 시간
    void OpenCAN();
    void BMS_ERROR_HANDLER(unsigned char, bool, Ui::MainWindow *);
    void saveToXML();
    bool save_flag =false;

public slots:
    void onTimer();
    


private slots:

    void on_SAVE_BUTTON_clicked();

    void on_input_string_textChanged(const QString &arg1);

private:
    Ui::MainWindow *ui;
    QTimer timer;
};
#endif // MAINWINDOW_H
