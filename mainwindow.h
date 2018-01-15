#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <QWidget>
#include <QTimer>

#include "rangeddata.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void calculate_kinematics();

private slots:

    void on_calculate_button_clicked();

public slots:

    void progress_update();
    void iter_req_update();

private:
    Ui::MainWindow *ui;

    QThread* kin_thread;
    QTimer*  update_timer;

    double inc = 0;
    double max_iterations = 0;
    double calculation_progress = 0;

    RangedData y_s0;
    RangedData init_velocity;
    RangedData theta;

    enum ETabIndex
    {
        TP_Kinematics = 0,
        TP_Results
    };

    // If input is whole add one and return.
    double if_int_inc(double x);
    bool near(double x, double y = 0, double tol = 0.001);
};

#endif // MAINWINDOW_H
