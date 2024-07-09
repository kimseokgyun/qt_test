#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);



    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    
    timer.start(50); //10

}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::onTimer()
{
    // Timer event handling code
}

void MainWindow::on_SAVE_BUTTON_clicked()
{
    // Handle save button click event
}

void MainWindow::on_input_string_textChanged(const QString &arg1)
{
    // Handle text change event
}