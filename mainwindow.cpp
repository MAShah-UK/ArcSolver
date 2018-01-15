#include <QtMath>
#include <QMessageBox>
#include <QTime>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "kinematics_thread.h"

using std::abs;
using std::pow;
using std::ceil;

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->main_tab->setTabEnabled(ETabIndex::TP_Results, false);

    connect(ui->min_y_s0_dspin,     SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->max_y_s0_dspin,     SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->min_velocity_dspin, SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->max_velocity_dspin, SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->min_angle_dspin,    SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->max_angle_dspin,    SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );
    connect(ui->increment_dspin,    SIGNAL(valueChanged(double)), this, SLOT(iter_req_update()) );

    kin_thread = new kinematics_thread<MainWindow>(*this, &MainWindow::calculate_kinematics);

    update_timer = new QTimer(this);
    connect(update_timer, SIGNAL(timeout()), this, SLOT(progress_update()));

    iter_req_update();
}

MainWindow::~MainWindow()
{
    kin_thread->wait();

    delete kin_thread;
    delete ui;
}

void MainWindow::on_calculate_button_clicked()
{
    ui->calculate_button->setEnabled(false);

    kin_thread->start();
    update_timer->start(50);
}

void MainWindow::progress_update()
{
    ui->calculation_pbar->setValue(calculation_progress);

    if (calculation_progress / 1000 >= 1)
        update_timer->stop();
}

void MainWindow::iter_req_update()
{

    y_s0.min          = ui->min_y_s0_dspin->value();
    y_s0.max          = ui->max_y_s0_dspin->value();
    init_velocity.min = ui->min_velocity_dspin->value();
    init_velocity.max = ui->max_velocity_dspin->value();
    theta.min         = ui->min_angle_dspin->value();
    theta.max         = ui->max_angle_dspin->value();

    inc = ui->increment_dspin->value();

    max_iterations = ceil( if_int_inc((y_s0.max - y_s0.min) / inc) ) *
                     ceil( if_int_inc((init_velocity.max - init_velocity.min) / inc) ) *
                     ceil( if_int_inc((theta.max - theta.min) / inc) );

    if (max_iterations <= 0)
        ui->iter_req_value_label->setText("Invalid");
    else
        ui->iter_req_value_label->setText(QString::number(max_iterations, 'f', 0));
}

double MainWindow::if_int_inc(double x)
{
    return near(remainder(x, 1)) ? (x + 1) : x;
}

bool MainWindow::near(double x, double y, double tol)
{
    return abs(x - y) <= tol;
}

void MainWindow::calculate_kinematics()
{
    double y_a = -9.81;

    // In this instance the min and max params refer to net and final positions respectively.
    RangedData x_s (ui->x_disp_over_net_dspin->value(), ui->x_final_disp_dspin->value());
    RangedData y_s (ui->y_disp_over_net_dspin->value(), 0);

    y_s0.curr = y_s0.min;
    init_velocity.curr = init_velocity.min;
    theta.curr = theta.min;

    QVector<RangedData*> RangedVars {&y_s0,
                                     &init_velocity,
                                     &theta};

    // Validation

    if (x_s.min >= x_s.max)
        { QMessageBox::warning(this, "Invalid X", "Ensure net < final.");   return; }

    if (y_s0.min > y_s0.max)
        { QMessageBox::warning(this, "Invalid S0,y", "Ensure min <= max."); return; }

    if (init_velocity.min > init_velocity.max)
        { QMessageBox::warning(this, "Invalid U", "Ensure min <= max.");    return; }

    if (theta.min > theta.max)
        { QMessageBox::warning(this, "Invalid θ", "Ensure min <= max.");    return; }

    // Calculations cache

    RangedData Iterations(0, max_iterations);

    double t1 = 0;
    double t2 = 0;

    double curr_y_t1_s = 0;
    double curr_y_t2_s = 0;

    double best_y_s0          = 0;
    double best_init_velocity = 0;
    double best_theta         = 0;
    double best_t1            = 0;
    double best_t2            = 0;
    double best_y_net         = 0;
    double best_y_ground      = -9999;

    // Begin calculations

    QTime time_taken;
    time_taken.start();

    bool NoMoreSolutions = false;
    while (!NoMoreSolutions)
    {
        calculation_progress = std::min(Iterations.curr / Iterations.max * 1000, 1000.0);

        // Have all possible values been exhausted?
        for (size_t i = 0; i < RangedVars.size(); ++i)
        {
            // Must add 0.001 as doubles are unreliable decimals and near wouldn't work.
            if (RangedVars[i]->curr > RangedVars[i]->max + 0.001)
            {
                if (i >= RangedVars.size() - 1)
                { NoMoreSolutions = true; break; }

                RangedVars[i]->curr = RangedVars[i]->min;
                RangedVars[i + 1]->curr += inc;
            }
        }

        t1 = x_s.min / (init_velocity.curr*qCos(qDegreesToRadians(theta.curr)) );
        curr_y_t1_s = y_s0.curr + init_velocity.curr*qSin(qDegreesToRadians(theta.curr))*t1 + 0.5*y_a*pow(t1, 2);

        t2 = x_s.max / (init_velocity.curr*qCos(qDegreesToRadians(theta.curr)) );
        curr_y_t2_s = y_s0.curr + init_velocity.curr*qSin(qDegreesToRadians(theta.curr))*t2 + 0.5*y_a*pow(t2, 2);

        if ( abs(y_s.min - curr_y_t1_s) + abs(y_s.max - curr_y_t2_s) <
             abs(y_s.min - best_y_net) + abs(y_s.max - best_y_ground) )

        {
            best_y_s0 = y_s0.curr;
            best_init_velocity = init_velocity.curr;
            best_theta = theta.curr;
            best_t1 = t1;
            best_t2 = t2;
            best_y_net = curr_y_t1_s;
            best_y_ground = curr_y_t2_s;
        }

        RangedVars[0]->curr += inc;

        if (!NoMoreSolutions) ++Iterations.curr;
    }

    ui->iterations_value_label    ->setText(QString::number(Iterations.curr, 'f', 0));
    ui->time_taken_value_label    ->setText(QString::number(time_taken.elapsed() / 1000.0, 'f', 2));
    ui->s0_y_value_label          ->setText(QString::number(best_y_s0, 'f', 2));
    ui->init_velocity_value_label ->setText(QString::number(best_init_velocity, 'f', 1));
    ui->theta_value_label         ->setText(QString::number(best_theta, 'f', 1));
    ui->s_net_value_label         ->setText(QString::number(best_y_net, 'f', 2));
    ui->s_final_value_label       ->setText(QString::number(best_y_ground, 'f', 2));
    ui->t_net_value_label         ->setText(QString::number(best_t1, 'f', 2));
    ui->t_final_value_label       ->setText(QString::number(best_t2, 'f', 2));

    ui->main_tab->setTabEnabled(ETabIndex::TP_Results, true);
    ui->main_tab->setCurrentIndex(ETabIndex::TP_Results);
    ui->calculate_button->setEnabled(true);

    // Set progress bar to 100%.
    calculation_progress = 1000;
}
