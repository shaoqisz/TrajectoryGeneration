#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <ruckig/ruckig.hpp>
#include <qdebug.h>
#include <QMessageBox>

using namespace ruckig;

namespace  {

    constexpr double DELTA_TIME = 0.01;

    double calculateMaxAcceleration(double distance, double maxSpeed, double maxJerk)
    {
        double T1 = 0.0;
        if (distance >= 2 * maxSpeed * pow(maxSpeed / maxJerk, 0.5)) {
            T1 = pow(maxSpeed / maxJerk, 0.5);
        } else {
            T1 = pow((distance / (2.0 * maxJerk)), 1.0f / 3);
        }
        double maxAcc = T1 * maxJerk;;
        return maxAcc;
    }

    double calculateMaxSpeed(double distance, double maxAcceleration, double maxJerk)
    {
        const double Vm = pow(maxAcceleration, 2) / maxJerk;
        const double ltVm = pow ( pow(distance, 2) * maxJerk / 4.0f, 1.0f/3.0f );
        if (ltVm < Vm) {
            return ltVm;
        }

        const double a = -(maxAcceleration / maxJerk);
        const double b = sqrt(pow(maxAcceleration / maxJerk, 2) + 4 * distance / maxAcceleration);
        const double c = 2 / maxAcceleration;
        const double gtVm = (a + b) / c;
        return gtVm;
    }

    double calculateTrajectoryTime(double distance, double maxSpeed, double maxAcceleration, double maxJerk)
    {
        const double S_Ref = 2 * pow(maxAcceleration, 3) / pow(maxJerk, 2);
        const double Vm = pow(maxAcceleration, 2) / maxJerk;
        //printf("Vm=%f\n", Vm);

        double T1, T2, T3, T4, T5, T6, T7;
        if (maxSpeed >= Vm)
        {
            if (distance <= S_Ref)
            {
                T1 = pow((distance / 2) * pow(maxJerk, 2), 1.0/3.0) / maxJerk;
                T2 = 0.0;
                T4 = 0.0;
            }
            else
            {
                if (distance < maxSpeed * (maxAcceleration / maxJerk + maxSpeed / maxAcceleration))
                {
                    T1 = maxAcceleration / maxJerk;
                    auto Vt = (-pow(maxAcceleration, 2) +
                        pow(pow(maxAcceleration, 4) + 4.0 * pow(maxJerk, 2) * maxAcceleration * distance, 0.5)) / (2.0 * maxJerk);
                    T2 = Vt / maxAcceleration - T1;
                    T4 = 0.0;
                }
                else
                {
                    T1 = maxAcceleration / maxJerk;
                    T2 = maxSpeed / maxAcceleration - T1;
                    T4 = (distance - maxSpeed * (maxAcceleration / maxJerk + maxSpeed / maxAcceleration)) / maxSpeed;
                }
            }
        }
        else
        {
            if (distance >= 2 * maxSpeed * pow(maxSpeed / maxJerk, 0.5))
            {
                T1 = pow(maxSpeed / maxJerk, 0.5);
                T2 = 0.0;
                T4 = (distance - 2.0 * maxSpeed * pow(maxSpeed / maxJerk, 0.5)) / maxSpeed;
            }
            else
            {
                T1 = pow((distance / (2.0 * maxJerk)), 1.0f / 3);
                T2 = 0.0;
                T4 = 0.0;
            }
        }

        T3 = T1;
        T5 = T1;
        T7 = T1;
        T6 = T2;

        return (T1 + T2 + T3 + T4 + T5 + T6 + T7);
    }
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    resize(800, 1024);

    ui->splitter->setStretchFactor(0, 5);
    ui->splitter->setStretchFactor(1, 5);
    ui->splitter->setStretchFactor(2, 5);

    connect(ui->spin_box_current_position, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_current_velocity, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_current_acceleration, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_target_position, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_target_velocity, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_target_acceleration, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_max_velocity, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_max_acceleration, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_max_jerk, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_stop_time, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);
    connect(ui->spin_box_stop_jerk, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,  &MainWindow::on_double_spin_valueChanged);


    connect(ui->spin_box_current_position, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_current_velocity, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_current_acceleration, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_target_position, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_target_velocity, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_target_acceleration, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_max_velocity, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_max_acceleration, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_max_jerk, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_stop_time, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);
    connect(ui->spin_box_stop_jerk, &QDoubleSpinBox::editingFinished, this,  &MainWindow::on_double_spin_editingFinished);

    connect(ui->combo_box_algorithm_type, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,  &MainWindow::combo_box_algorithm_type_changed);

//    connect(ui->btn_apply_max_vel, &QPushButton::clicked, this, &MainWindow::on_btn_apply_max_vel_pressed);
//    connect(ui->btn_apply_max_accel, &QPushButton::clicked, this, &MainWindow::on_btn_apply_max_accel_pressed);


//    generateTrajectory();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_check_box_show_jerk_stateChanged(int)
{
    updateYRange();
}


void MainWindow::on_check_box_show_trace_stateChanged(int)
{

}


void MainWindow::on_action_version_triggered(bool)
{
    showVersionDialog();
}


void MainWindow::showVersionDialog()
{
    QDialog dialog(this);
    dialog.setWindowTitle("Version");
    dialog.resize(QSize(300, 100));
    dialog.setWindowFlag(Qt::WindowContextHelpButtonHint, false);

    QLabel *label = new QLabel(&dialog);
    label->setText("v2.1");
    label->setAlignment(Qt::AlignCenter);

    QGridLayout *gridLayout = new QGridLayout(&dialog);
    gridLayout->addWidget(label, 0, 0, 1, 1);

    dialog.exec();
}
void MainWindow::showValidateInputError()
{
    QMessageBox::warning(this, tr("Warning"), tr("Unable to generate trajectory, please check your input parameters."), QMessageBox::Ok);
}

void MainWindow::combo_box_algorithm_type_changed(int)
{
    if (ui->check_box_quick_calculate->checkState() == Qt::Checked) {
        generateTrajectory();
    }
}

void MainWindow::on_double_spin_editingFinished()
{
    std::cout << "on_double_spin_editingFinished" << std::endl;
    if (ui->check_box_quick_calculate->checkState() == Qt::Checked) {
        generateTrajectory();
    }
}

void MainWindow::on_double_spin_valueChanged(double)
{
    std::cout << "on_double_spin_valueChanged" << std::endl;
    if (ui->check_box_quick_calculate->checkState() == Qt::Checked) {
        generateTrajectory();
    }
}

void MainWindow::on_btn_generate_trajectory_pressed()
{
    generateTrajectory();
}

void MainWindow::on_btn_clear_trajectory_pressed()
{
    ui->trajectoryView->clear();
    ui->text_logs->clear();
    ui->line_trajectory_time->clear();
}


void MainWindow::on_btn_apply_max_vel_pressed()
{
    std::cout << "on_btn_apply_max_vel_pressed" << std::endl;

    ui->spin_box_max_velocity->setValue(ui->line_actual_max_velocity->text().toDouble());
}

void MainWindow::on_btn_apply_max_accel_pressed()
{
    std::cout << "on_btn_apply_max_accel_pressed" << std::endl;
    ui->spin_box_max_acceleration->setValue(ui->line_actual_max_accel->text().toDouble());
}


void MainWindow::updateYRange()
{
    ui->trajectoryView->setJerkVisible(ui->check_box_show_jerk->checkState() == Qt::Checked ? true : false);

    int maxY = 0;
    auto pickLessOne = [&](double value) {
        auto absValue = abs(value);
        if (absValue > maxY) maxY = absValue;
    };

    pickLessOne(ui->spin_box_target_position->value());
    pickLessOne(ui->spin_box_max_velocity->value());
    pickLessOne(ui->spin_box_max_acceleration->value());
    pickLessOne(ui->spin_box_current_position->value());
    pickLessOne(ui->spin_box_current_velocity->value());
    pickLessOne(ui->spin_box_current_acceleration->value());

    if (ui->check_box_show_jerk->checkState() == Qt::Checked) {
        pickLessOne(ui->spin_box_max_jerk->value());
        pickLessOne(ui->spin_box_stop_jerk->value());

    }
    ui->trajectoryView->setYRange(-maxY*1.1, maxY*1.1);
}


void MainWindow::generateTrajectory()
{
    /////////////////////////
    const auto distance = abs(ui->spin_box_target_position->value()-ui->spin_box_current_position->value());
    const auto trajectoryTime = calculateTrajectoryTime(distance, ui->spin_box_max_velocity->value(), ui->spin_box_max_acceleration->value(), ui->spin_box_max_jerk->value());

    const auto actualMaxAccel = calculateMaxAcceleration(distance, ui->spin_box_max_velocity->value(), ui->spin_box_max_jerk->value());
    const auto actualMaxSpeed = calculateMaxSpeed(distance, ui->spin_box_max_acceleration->value(), ui->spin_box_max_jerk->value());

    ui->line_actual_max_accel->setText(QString("%1").arg(QString::number(actualMaxAccel,'f', 6)));
    ui->line_actual_max_velocity->setText(QString("%1").arg(QString::number(actualMaxSpeed,'f', 6)));

    if (ui->spin_box_max_velocity->text().toDouble() - actualMaxSpeed > 0.00001) {
        ui->line_actual_max_velocity->setStyleSheet("color: red;  background-color: white");
        ui->btn_apply_max_vel->setEnabled(true);
    } else {
        ui->line_actual_max_velocity->setStyleSheet("color: black;  background-color: white");
        ui->btn_apply_max_vel->setDisabled(true);
    }
    if (ui->spin_box_max_acceleration->text().toDouble() - actualMaxAccel > 0.00001) {
        ui->line_actual_max_accel->setStyleSheet("color: red;  background-color: white");
        ui->btn_apply_max_accel->setEnabled(true);
    } else {
        ui->line_actual_max_accel->setStyleSheet("color: black;  background-color: white");
        ui->btn_apply_max_accel->setDisabled(true);
    }

    ui->trajectoryView->setXRange(0, trajectoryTime*1.02);

    /////////////////////////

    std::cout << "index=" << ui->combo_box_algorithm_type->currentIndex() << std::endl;

    if (ui->combo_box_algorithm_type->currentIndex() == 0) {
        runOnlineTrajectoryGeneration();

        ui->spin_box_stop_time->setEnabled(false);
        ui->spin_box_stop_jerk->setEnabled(false);
        ui->spin_box_max_velocity->setEnabled(true);

        ui->spin_box_target_position->setEnabled(true);
        ui->spin_box_target_position->setEnabled(true);

    } else if (ui->combo_box_algorithm_type->currentIndex() == 1) {
        runOfflineTrajectoryGeneration();

        ui->spin_box_stop_time->setEnabled(false);
        ui->spin_box_stop_jerk->setEnabled(false);
        ui->spin_box_max_velocity->setEnabled(true);

        ui->spin_box_target_position->setEnabled(true);
        ui->spin_box_target_position->setEnabled(true);

    } else if (ui->combo_box_algorithm_type->currentIndex() == 2) {
        runVelocityTrajectoryGeneration();

        ui->spin_box_stop_time->setEnabled(false);
        ui->spin_box_stop_jerk->setEnabled(false);
        ui->spin_box_max_velocity->setEnabled(false);

        ui->spin_box_target_position->setEnabled(false);
        ui->spin_box_target_position->setEnabled(false);


    } else if (ui->combo_box_algorithm_type->currentIndex() == 3) {
        runStopTrajectoryGeneration();

        ui->spin_box_stop_time->setEnabled(true);
        ui->spin_box_stop_jerk->setEnabled(true);
        ui->spin_box_max_velocity->setEnabled(true);

        ui->spin_box_target_position->setEnabled(true);
        ui->spin_box_target_position->setEnabled(true);
    }
}

void MainWindow::runOnlineTrajectoryGeneration()
{
    qDebug() << __FUNCTION__;

    on_btn_clear_trajectory_pressed();

    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<1> otg{ DELTA_TIME };  // control cycle
    InputParameter<1> input;
    OutputParameter<1> output;

    // Set input parameters
    input.current_position      = { ui->spin_box_current_position->value() };
    input.current_velocity      = { ui->spin_box_current_velocity->value() };
    input.current_acceleration  = { ui->spin_box_current_acceleration->value() };

    input.target_position       = { ui->spin_box_target_position->value()};
    input.target_velocity       = { ui->spin_box_target_velocity->value() };
    input.target_acceleration   = { ui->spin_box_target_acceleration->value() };

    input.max_velocity          = { ui->spin_box_max_velocity->value() };
    input.max_acceleration      = { ui->spin_box_max_acceleration->value() };
    input.max_jerk              = { ui->spin_box_max_jerk->value() };
    try {
        otg.validate_input(input);
    } catch (RuckigError &err) {
        std::cout << err.what() << std::endl;
        showValidateInputError();
        return;
    }

    ui->text_logs->append("Time\tPosition\tVelocity\tAcceleration\tJerk");
    const auto start = std::chrono::high_resolution_clock::now();
    while (otg.update(input, output) == Result::Working) {
//        std::cout << std::fixed << output.time << "\t" << output.new_position[0] << "\t" << output.new_velocity[0] << "\t" << output.new_acceleration[0] << "\t" << output.new_jerk[0] << std::endl;
        output.pass_to_input(input);

        ui->trajectoryView->appendData(output.time,
                                       output.new_position[0],
                                       output.new_velocity[0],
                                       output.new_acceleration[0],
                                       output.new_jerk[0]);

        ui->trajectoryView->setMaxVelocityAccel(output.time, input.max_velocity[0], input.max_acceleration[0]);

        if (ui->check_box_show_trace->checkState() == Qt::Checked) {
            ui->text_logs->append(QString("%1\t%2\t%3\t%4\t%5")
                                  .arg(output.time).arg(output.new_position[0]).arg(output.new_velocity[0])
                                  .arg(output.new_acceleration[0]).arg(output.new_jerk[0]));
        }

    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

//    ui->trajectoryView->setXRange(0, output.trajectory.get_duration() * 1.02);
    updateYRange();

    ui->line_trajectory_time->setText(QString("%1").arg(output.trajectory.get_duration()));

    ui->text_logs->append(QString("Online Trajectory duration: %1 s").arg(output.trajectory.get_duration()));
    ui->text_logs->append(QString("Online Calculating Time: %1 us").arg(double(duration.count())));
}

void MainWindow::runOfflineTrajectoryGeneration()
{
    qDebug() << __FUNCTION__;

    on_btn_clear_trajectory_pressed();

    // Create input parameters
    InputParameter<1> input;
    // Set input parameters
    input.current_position      = { ui->spin_box_current_position->value() };
    input.current_velocity      = { ui->spin_box_current_velocity->value() };
    input.current_acceleration  = { ui->spin_box_current_acceleration->value() };

    input.target_position       = { ui->spin_box_target_position->value()};
    input.target_velocity       = { ui->spin_box_target_velocity->value() };
    input.target_acceleration   = { ui->spin_box_target_acceleration->value() };

    input.max_velocity          = { ui->spin_box_max_velocity->value() };
    input.max_acceleration      = { ui->spin_box_max_acceleration->value() };
    input.max_jerk              = { ui->spin_box_max_jerk->value() };

    // We don't need to pass the control rate (cycle time) when using only offline features
    Ruckig<1> otg;
    Trajectory<1> trajectory;

    try {
        otg.validate_input(input);
    } catch (RuckigError &err) {
        std::cout << err.what() << std::endl;
        showValidateInputError();
        return;
    }


    const auto start = std::chrono::high_resolution_clock::now();
    // Calculate the trajectory in an offline manner (outside of the control loop)
    Result result = otg.calculate(input, trajectory);
    if (result == Result::ErrorInvalidInput) {
        std::cout << "Invalid input!" << std::endl;
        return;
    }

    // Get duration of the trajectory
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    ui->text_logs->append("Time\tPosition\tVelocity\tAcceleration\tJerk");
    for (double t = 0.0; t < trajectory.get_duration(); t+= DELTA_TIME) {
        double new_time{ t };
        std::array<double, 1> new_position, new_velocity, new_acceleration, new_jerk;
        size_t new_section = 0;
        trajectory.at_time(new_time, new_position, new_velocity, new_acceleration, new_jerk, new_section);

        ui->trajectoryView->appendData(new_time,
                                       new_position[0],
                                       new_velocity[0],
                                       new_acceleration[0],
                                       new_jerk[0]);

        ui->trajectoryView->setMaxVelocityAccel(new_time, input.max_velocity[0], input.max_acceleration[0]);

        if (ui->check_box_show_trace->checkState() == Qt::Checked) {
            ui->text_logs->append(QString("%1\t%2\t%3\t%4\t%5")
                                  .arg(new_time).arg(new_position[0]).arg(new_velocity[0])
                                  .arg(new_acceleration[0]).arg(new_jerk[0]));
        }
    }

//    ui->trajectoryView->setXRange(0, trajectory.get_duration()*1.02);
    updateYRange();

    ui->line_trajectory_time->setText(QString("%1").arg(trajectory.get_duration()));

    ui->text_logs->append(QString("Offline Trajectory duration: %1 s").arg(trajectory.get_duration()));
    ui->text_logs->append(QString("Offline Calculating Time: %1 us").arg(double(duration.count())));
}

void MainWindow::runVelocityTrajectoryGeneration()
{
    qDebug() << __FUNCTION__;

    on_btn_clear_trajectory_pressed();

    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<1> otg{ DELTA_TIME };  // control cycle
    InputParameter<1> input;
    OutputParameter<1> output;

    // Set input parameters and velocity control interface
    input.control_interface = ControlInterface::Velocity;

    input.current_position      = { ui->spin_box_current_position->value() };
    input.current_velocity      = { ui->spin_box_current_velocity->value() };
    input.current_acceleration  = { ui->spin_box_current_acceleration->value() };

    input.target_velocity       = { ui->spin_box_target_velocity->value() };
    input.target_acceleration   = { ui->spin_box_target_acceleration->value() };

    input.max_acceleration      = { ui->spin_box_max_acceleration->value() };
    input.max_jerk              = { ui->spin_box_max_jerk->value() };

    try {
        otg.validate_input(input);
    } catch (RuckigError &err) {
        std::cout << err.what() << std::endl;
        showValidateInputError();
        return;
    }


    // Generate the trajectory within the control loop
    ui->text_logs->append("Time\tPosition\tVelocity\tAcceleration\tJerk");

    const auto start = std::chrono::high_resolution_clock::now();
    while (otg.update(input, output) == Result::Working) {
        std::cout << std::fixed << output.time << "\t" << output.new_position[0] << "\t" << output.new_velocity[0] << "\t" << output.new_acceleration[0] << std::endl;
        output.pass_to_input(input);


        ui->trajectoryView->appendData(output.time,
                                       output.new_position[0],
                                       output.new_velocity[0],
                                       output.new_acceleration[0],
                                       output.new_jerk[0]);
        ui->trajectoryView->setMaxVelocityAccel(output.time, input.max_velocity[0], input.max_acceleration[0]);

        if (ui->check_box_show_trace->checkState() == Qt::Checked) {
            ui->text_logs->append(QString("%1\t%2\t%3\t%4\t%5")
                                  .arg(output.time).arg(output.new_position[0]).arg(output.new_velocity[0])
                                  .arg(output.new_acceleration[0]).arg(output.new_jerk[0]));
        }
    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

//    ui->trajectoryView->setXRange(0, output.trajectory.get_duration()*1.02);
    updateYRange();

    ui->line_trajectory_time->setText(QString("%1").arg(output.trajectory.get_duration()));

    ui->text_logs->append(QString("Velocity Trajectory duration: %1 s").arg(output.trajectory.get_duration()));
    ui->text_logs->append(QString("Velocity Calculating Time: %1 us").arg(double(duration.count())));
}


void MainWindow::runStopTrajectoryGeneration()
{
    qDebug() << __FUNCTION__;

    on_btn_clear_trajectory_pressed();

    // Create instances: the ruckig otg as well as input and output parameters
    Ruckig<1> otg { DELTA_TIME };
    InputParameter<1> input;
    OutputParameter<1> output;

    // Set input parameters
    input.current_position      = { ui->spin_box_current_position->value() };
    input.current_velocity      = { ui->spin_box_current_velocity->value() };
    input.current_acceleration  = { ui->spin_box_current_acceleration->value() };

    input.target_position       = { ui->spin_box_target_position->value()};
    input.target_velocity       = { ui->spin_box_target_velocity->value() };
    input.target_acceleration   = { ui->spin_box_target_acceleration->value() };

    input.max_velocity          = { ui->spin_box_max_velocity->value() };
    input.max_acceleration      = { ui->spin_box_max_acceleration->value() };
    input.max_jerk              = { ui->spin_box_max_jerk->value() };


    try {
        otg.validate_input(input);
    } catch (RuckigError &err) {
        std::cout << err.what() << std::endl;
        showValidateInputError();
        return;
    }


    // Generate the trajectory within the control loop
    ui->text_logs->append("Time\tPosition\tVelocity\tAcceleration\tJerk");
    const auto start = std::chrono::high_resolution_clock::now();

    bool on_stop_trajectory {false};
    double stop_trajectory_time = 0.0;
    while (otg.update(input, output) == Result::Working) {

        ui->trajectoryView->appendData(output.time + stop_trajectory_time,
                                       output.new_position[0],
                                       output.new_velocity[0],
                                       output.new_acceleration[0],
                                       output.new_jerk[0]);

        ui->trajectoryView->setMaxVelocityAccel(output.time + stop_trajectory_time, input.max_velocity[0], input.max_acceleration[0]);

        if (ui->check_box_show_trace->checkState() == Qt::Checked) {
            ui->text_logs->append(QString("%1\t%2\t%3\t%4\t%5")
                                  .arg(output.time + stop_trajectory_time).arg(output.new_position[0]).arg(output.new_velocity[0])
                                  .arg(output.new_acceleration[0]).arg(output.new_jerk[0]));
        }

      // Activate stop trajectory after 1s
      if (output.time >= ui->spin_box_stop_time->value() && !on_stop_trajectory) {
          std::cout << "Stop immediately." << std::endl;
          on_stop_trajectory = true;
          stop_trajectory_time = output.time;

          // Synchronization is disabled so that each DoF stops as fast as possible independently
          input.control_interface = ControlInterface::Velocity;
          input.synchronization = Synchronization::None;
          input.target_velocity = {0.0};
          input.target_acceleration = {0.0};
          input.max_jerk = {ui->spin_box_stop_jerk->value()};
      }

      output.pass_to_input(input);

    }

    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

//    ui->trajectoryView->setXRange(0, (output.trajectory.get_duration() + stop_trajectory_time) * 1.02);
    updateYRange();

    ui->line_trajectory_time->setText(QString("%1").arg(output.trajectory.get_duration() + stop_trajectory_time));

    ui->text_logs->append(QString("Velocity Trajectory duration: %1 s").arg(output.trajectory.get_duration() + stop_trajectory_time));
    ui->text_logs->append(QString("Velocity Calculating Time: %1 us").arg(double(duration.count())));
}


