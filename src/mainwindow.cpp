#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //for debug
    can_status=0;

    state_station = 0;
    state_amr = 0;

    station_limit_sensor = 0;

    station_current_charger = 0;
    station_current_contact = 0;
    station_voltage_charger = 0;
    station_voltage_contact = 0;


    amr_current_contact = 0;
    amr_current_battery = 0;
    amr_voltage_contact = 0;
    amr_voltage_battery = 0;

;



    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    
    timer.start(50); //10

}

MainWindow::~MainWindow()
{
    delete ui;
}





void MainWindow::onTimer(){



    if(can_status ==1){
        SetLEColor(ui->CAN_STATE_IDLE, "green");
    }
    else {
        SetLEColor(ui->CAN_STATE_IDLE, "red");
    }

    if(station_limit_sensor == 1){
        SetLEColor(ui->LE_STATION_LIMIT, "white");
    }else{
        SetLEColor(ui->LE_STATION_LIMIT, "blue");
    }

    ui->LE_STATION_CURRENT_1->setText(QString().sprintf("%.1f", station_current_charger));
    ui->LE_STATION_CURRENT_2->setText(QString().sprintf("%.1f", station_current_contact));

    ui->LE_STATION_VOLTAGE_1->setText(QString().sprintf("%d", station_voltage_charger));
    ui->LE_STATION_VOLTAGE_2->setText(QString().sprintf("%d", station_voltage_contact));

 
    ui->LE_AMR_CURRENT_1->setText(QString().sprintf("%.1f", amr_current_contact));
    ui->LE_AMR_CURRENT_2->setText(QString().sprintf("%.1f", amr_current_battery));

    ui->LE_AMR_VOLTAGE_1->setText(QString().sprintf("%d", amr_voltage_contact));
    ui->LE_AMR_VOLTAGE_2->setText(QString().sprintf("%d", amr_voltage_battery));


    //bms error handler

    for (int i=0; i<=6; i++){

        bool bitStatus = (BMS_STATUS >>i) & 1;
        BMS_ERROR_HANDLER(i,bitStatus,ui);

    }

    ui->BMS_VOLTAGE->setText(QString().sprintf("%.2f", BMS_VOLTAGE));
    ui->BMS_CURRENT->setText(QString().sprintf("%.2f", BMS_CURRENT));
    ui->BMS_TTF->setText(QString().sprintf("%d", BMS_TTF));
    ui->BMS_TTE->setText(QString().sprintf("%d", BMS_TTE));
    ui->BMS_SOC->setText(QString().sprintf("%d", BMS_SOC));
    ui->BMS_SOH->setText(QString().sprintf("%d", BMS_SOH));

    ui->BMS_RC->setText(QString().sprintf("%.2f", BMS_RC));
    ui->BMS_AE->setText(QString().sprintf("%.1f", BMS_AE));
    ui->BMS_TEMP->setText(QString().sprintf("%.1f", BMS_TEMP));



    SetLEColor(ui->LE_STATE_STATION_IDLE, "white");
    SetLEColor(ui->LE_STATE_STATION_5V_ON, "white");
    SetLEColor(ui->LE_STATE_STATION_CHARGER_ON, "white");
    SetLEColor(ui->LE_STATE_STATION_CHARGING, "white");
    SetLEColor(ui->LE_STATE_STATION_CHARGER_OFF, "white");
    SetLEColor(ui->LE_STATE_STATION_FAIL, "white");

    if(state_station == 0){
        SetLEColor(ui->LE_STATE_STATION_IDLE, "green");
    }else if(state_station == 1){
        SetLEColor(ui->LE_STATE_STATION_5V_ON, "green");
    }else if(state_station == 2){
        SetLEColor(ui->LE_STATE_STATION_CHARGER_ON, "green");
    }else if(state_station == 3){
        SetLEColor(ui->LE_STATE_STATION_CHARGING, "green");
    }else if(state_station == 4){
        SetLEColor(ui->LE_STATE_STATION_CHARGER_OFF, "green");
    }else if(state_station == 5){
        SetLEColor(ui->LE_STATE_STATION_FAIL, "green");
    }


    SetLEColor(ui->LE_STATE_AMR_IDLE, "white");
    SetLEColor(ui->LE_STATE_AMR_TRIG_TO_CHARGE, "white");
    SetLEColor(ui->LE_STATE_AMR_BATTERY_ON, "white");
    SetLEColor(ui->LE_STATE_AMR_CHARGING, "white");
    SetLEColor(ui->LE_STATE_AMR_TRIG_TO_STOP, "white");
    SetLEColor(ui->LE_STATE_AMR_FAIL, "white");

    if(state_amr == 0){
        SetLEColor(ui->LE_STATE_AMR_IDLE, "green");
    }else if(state_amr == 1){
        SetLEColor(ui->LE_STATE_AMR_TRIG_TO_CHARGE, "green");
    }else if(state_amr == 2){
        SetLEColor(ui->LE_STATE_AMR_BATTERY_ON, "green");
    }else if(state_amr == 3){
        SetLEColor(ui->LE_STATE_AMR_CHARGING, "green");
    }else if(state_amr == 4){
        SetLEColor(ui->LE_STATE_AMR_TRIG_TO_STOP, "green");
    }else if(state_amr == 5){
        SetLEColor(ui->LE_STATE_AMR_FAIL, "green");
    }



}

void MainWindow::BMS_ERROR_HANDLER(unsigned char num, bool state, Ui::MainWindow *ui){
    
    switch(num){
  
        case 0:
            {
                if(state){SetLEColor(ui->BMS_ERROR_OV, "red");}
                else{SetLEColor(ui->BMS_ERROR_OV, "green");}
                break;}

        case 1:
            {
                if(state){SetLEColor(ui->BMS_ERROR_LV, "red");}
                else{SetLEColor(ui->BMS_ERROR_LV, "green");}
                break;}
        case 2:
            {
                if(state){SetLEColor(ui->BMS_ERROR_OCC, "red");}
                else{SetLEColor(ui->BMS_ERROR_OCC, "green");}
                break;}
        case 3:
            {
                if(state){SetLEColor(ui->BMS_ERROR_LDC, "red");}
                else{SetLEColor(ui->BMS_ERROR_LDC, "green");}
                break;}
        case 4:
            {
                if(state){SetLEColor(ui->BMS_ERROR_OT, "red");}
                else{SetLEColor(ui->BMS_ERROR_OT, "green");}                
                break;}
        case 5:
            {
                if(state){SetLEColor(ui->BMS_ERROR_LT, "red");}
                else{SetLEColor(ui->BMS_ERROR_LT, "green");}               
                break;}
        case 6:
            {
                 if(state){SetLEColor(ui->BMS_ERROR_BMS_ERROR, "red");}
                else{SetLEColor(ui->BMS_ERROR_BMS_ERROR, "green");}               
                break;}


    }

}


void MainWindow::on_SAVE_BUTTON_clicked()
{
    save_flag =!(save_flag);
    qDebug() << "click";

    if(save_flag){    startTime = QDateTime::currentDateTime(); }// 타이머 시작 시간 초기화}
}


void MainWindow::on_input_string_textChanged(const QString &arg1)
{

    // savefilename_ = arg1.toStdString();
    savefilename_ = arg1;
}
