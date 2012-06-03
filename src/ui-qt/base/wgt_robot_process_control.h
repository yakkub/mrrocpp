#ifndef WGT_ROBOT_PROCESS_CONTROL_H
#define WGT_ROBOT_PROCESS_CONTROL_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_robot_process_control.h"
#include "wgt_base.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class UiRobot;
class AllRobots;

}
}
}

class wgt_robot_process_control : public wgt_base
{
	Q_OBJECT

public:
	wgt_robot_process_control(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	~wgt_robot_process_control();

	void process_control_window_init();

	void my_open(bool set_on_top = false);

	Ui::wgt_robot_process_controlClass * get_ui();
	void block_ecp_trigger_widgets();
	void unblock_ecp_trigger_widgets();
	virtual void add_button(QPushButton *button, int row, int space)
	{
	}
	virtual void setup_ui()
	{
	}

private:
	Ui::wgt_robot_process_controlClass* ui;

	mrrocpp::ui::common::UiRobot *robot;

	// aktualizacja ustawien przyciskow
	void init();

	signals:
	void process_control_window_init_signal();

public slots:
	void process_control_window_init_slot();

private slots:
	// ECP
	void on_ecp_trigger_pushButton_clicked();

	// Reader
	void on_reader_start_pushButton_clicked();
	void on_reader_stop_pushButton_clicked();
	void on_reader_trigger_pushButton_clicked();

};

#endif // WGT_PROCESS_CONTROL_H
