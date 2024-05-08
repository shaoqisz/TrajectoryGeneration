#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:    
    void on_btn_generate_trajectory_pressed();
    void on_btn_clear_trajectory_pressed();

    void on_double_spin_editingFinished();
    void on_double_spin_valueChanged(double);

    void on_check_box_show_jerk_stateChanged(int);
    void on_check_box_show_trace_stateChanged(int);

    void on_action_version_triggered(bool checked);

    void combo_box_algorithm_type_changed(int);

    void on_btn_apply_max_vel_pressed();

    void on_btn_apply_max_accel_pressed();

private:
    void updateYRange();

    void runOnlineTrajectoryGeneration();
    void runOfflineTrajectoryGeneration();
    void runVelocityTrajectoryGeneration();
    void runStopTrajectoryGeneration();

    void generateTrajectory();

    void showVersionDialog();
    void showValidateInputError();

private:
    Ui::MainWindow  *ui;
};
#endif // MAINWINDOW_H

