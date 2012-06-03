#include "wgt_bird_hand_command.h"
#include "ui_wgt_bird_hand_command.h"
#include "ui_ecp_r_bird_hand.h"
#include "ui_r_bird_hand.h"
#include "../../robot/bird_hand/dp_bird_hand.h"
#include "../../robot/bird_hand/const_bird_hand.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common012.h"

#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"
#include <QAbstractButton>
#include <QCheckBox>

wgt_bird_hand_command::wgt_bird_hand_command(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		wgt_base(_widget_label, _interface, _robot, parent), ui(new Ui::wgt_bird_hand_commandClass())
{
	//ui = new Ui::wgt_bird_hand_commandClass::wgt_bird_hand_commandClass();
	ui->setupUi(this);
	robot = dynamic_cast <mrrocpp::ui::bird_hand::UiRobot *>(_robot);

	connect(this, SIGNAL(init_and_copy_signal()), this, SLOT(init_and_copy_slot()), Qt::QueuedConnection);

	// zacienianie kontrolek momentu dla obrotow palcy
	ui->doubleSpinBox_curtor_index_0->hide();
	ui->doubleSpinBox_curtor_ring_0->hide();
	ui->doubleSpinBox_destor_index_0->hide();
	ui->doubleSpinBox_destor_ring_0->hide();

	// budowanie wektoro kontrolek

	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_thumb_0);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_thumb_1);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_0);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_1);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_index_2);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_0);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_1);
	doubleSpinBox_curpos_Vector.append(ui->doubleSpinBox_curpos_ring_2);

	desired_pos_spin_box.append(ui->doubleSpinBox_despos_thumb_0);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_thumb_1);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_index_0);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_index_1);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_index_2);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_ring_0);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_ring_1);
	desired_pos_spin_box.append(ui->doubleSpinBox_despos_ring_2);

	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_thumb_0);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_thumb_1);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_0);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_1);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_index_2);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_0);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_1);
	doubleSpinBox_destor_Vector.append(ui->doubleSpinBox_destor_ring_2);

	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_thumb_0);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_thumb_1);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_0);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_1);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_index_2);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_0);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_1);
	doubleSpinBox_curtor_Vector.append(ui->doubleSpinBox_curtor_ring_2);

	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_thumb_0);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_thumb_1);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_0);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_1);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_index_2);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_0);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_1);
	doubleSpinBox_rdamp_Vector.append(ui->doubleSpinBox_rdamp_ring_2);

	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_thumb_0);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_thumb_1);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_0);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_1);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_index_2);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_0);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_1);
	doubleSpinBox_mcur_Vector.append(ui->doubleSpinBox_mcur_ring_2);

	buttonGroup_Vector.append(ui->buttonGroup_thumb_0);
	buttonGroup_Vector.append(ui->buttonGroup_thumb_1);
	buttonGroup_Vector.append(ui->buttonGroup_index_0);
	buttonGroup_Vector.append(ui->buttonGroup_index_1);
	buttonGroup_Vector.append(ui->buttonGroup_index_2);
	buttonGroup_Vector.append(ui->buttonGroup_ring_0);
	buttonGroup_Vector.append(ui->buttonGroup_ring_1);
	buttonGroup_Vector.append(ui->buttonGroup_ring_2);

	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_thumb_0);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_thumb_1);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_0);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_1);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_index_2);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_0);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_1);
	checkboxButtonGroup_Vector.append(ui->buttonGroup_checkbox_ring_2);

	for (int i = 0; i < robot->number_of_servos; i++) {
		checkboxButtonGroup_Vector[i]->setExclusive(false);

		doubleSpinBox_curtor_Vector[i]->setEnabled(false);
		doubleSpinBox_mcur_Vector[i]->setEnabled(false);
		doubleSpinBox_curpos_Vector[i]->setEnabled(false);

		QList <QAbstractButton*> buttons_in_group = checkboxButtonGroup_Vector[i]->buttons();
		for (int j = 0; j < buttons_in_group.size(); j++) {
			buttons_in_group[j]->setEnabled(false);
		}
	}

	current_profile_type = lib::bird_hand::MACROSTEP_POSITION_INCREMENT;

}

wgt_bird_hand_command::~wgt_bird_hand_command()
{
	//  delete ui;
}

void wgt_bird_hand_command::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	init_and_copy_slot();
}

void wgt_bird_hand_command::synchro_depended_init()
{
	emit synchro_depended_init_signal();
}

void wgt_bird_hand_command::init_and_copy()
{
	emit init_and_copy_signal();
}

void wgt_bird_hand_command::on_pushButton_read_clicked()
{

	set_status();
}

void wgt_bird_hand_command::init_and_copy_slot()
{
	set_status();
	copy_command();
}

void wgt_bird_hand_command::on_pushButton_copy_clicked()
{
	copy_command();
}

void wgt_bird_hand_command::on_pushButton_clear_all_clicked()
{
	for (int i = 0; i < robot->number_of_servos; i++) {
		desired_pos_spin_box[i]->setValue(0);
		doubleSpinBox_destor_Vector[i]->setValue(0);
		doubleSpinBox_rdamp_Vector[i]->setValue(0);
	}
}

void wgt_bird_hand_command::on_pushButton_change_command_type_all_clicked()
{

	int button_number;

	// zmien profil i przypisz kolumne przycisku do zaznaczenia
	switch (current_profile_type)
	{
		case lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION:
			current_profile_type = lib::bird_hand::MACROSTEP_POSITION_INCREMENT;
			button_number = 1;
			break;
		case lib::bird_hand::MACROSTEP_POSITION_INCREMENT:
			current_profile_type = lib::bird_hand::SIGLE_STEP_POSTION_INCREMENT;
			button_number = 2;
			break;
		case lib::bird_hand::SIGLE_STEP_POSTION_INCREMENT:
			current_profile_type = lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION;
			button_number = 0;
			break;
		default:
			break;
	}

	// ustaw odpowiednie przyciski
	for (int i = 0; i < robot->number_of_servos; i++) {

		QList <QAbstractButton*> buttons_in_group = buttonGroup_Vector[i]->buttons();

		buttons_in_group[button_number]->toggle();

	}
}

void wgt_bird_hand_command::get_command()
{
	try {

		//lib::bird_hand::command &bhcs = robot->ui_ecp_robot->bird_hand_command_data_port.data;

		mrrocpp::lib::bird_hand::command &bhcs = robot->ui_ecp_robot->the_robot->bird_hand_command_data_port.data;

		// odczyt ilosci krokow i ecp_query step

		bhcs.motion_steps = ui->spinBox_motion_steps->value();
		bhcs.ecp_query_step = bhcs.motion_steps - ui->spinBox_query_step->value();

		for (int i = 0; i < robot->number_of_servos; i++) {
			get_finger_command(i);
			get_variant_finger_command(i);
		}

		//std::stringstream ss(std::stringstream::in | std::stringstream::out);
		/*
		 ss << bhcs.index_f[0].profile_type << " " << bhcs.motion_steps << "  "
		 << bhcs.ecp_query_step;
		 */
		/*
		 ss << bhcs.index_f[0].desired_position << " "
		 << bhcs.index_f[0].desired_torque << "  "
		 << bhcs.index_f[0].reciprocal_of_damping;

		 interface.ui_msg->message(ss.str().c_str());
		 */
		robot->ui_ecp_robot->the_robot->bird_hand_command_data_port.set();
		robot->ui_ecp_robot->execute_motion();

	} // end try
	CATCH_SECTION_UI_PTR
}

void wgt_bird_hand_command::set_status()
{

	joint_status.clear();
	joint_command.clear();

	mrrocpp::lib::bird_hand::status &bhsrs =
			robot->ui_ecp_robot->the_robot->bird_hand_status_reply_data_request_port.data;

	joint_status.append(&bhsrs.thumb_f[0]);
	joint_status.append(&bhsrs.thumb_f[1]);
	joint_status.append(&bhsrs.index_f[0]);
	joint_status.append(&bhsrs.index_f[1]);
	joint_status.append(&bhsrs.index_f[2]);
	joint_status.append(&bhsrs.ring_f[0]);
	joint_status.append(&bhsrs.ring_f[1]);
	joint_status.append(&bhsrs.ring_f[2]);

	mrrocpp::lib::bird_hand::command &bhcs = robot->ui_ecp_robot->the_robot->bird_hand_command_data_port.data;

	joint_command.append(&bhcs.thumb_f[0]);
	joint_command.append(&bhcs.thumb_f[1]);
	joint_command.append(&bhcs.index_f[0]);
	joint_command.append(&bhcs.index_f[1]);
	joint_command.append(&bhcs.index_f[2]);
	joint_command.append(&bhcs.ring_f[0]);
	joint_command.append(&bhcs.ring_f[1]);
	joint_command.append(&bhcs.ring_f[2]);

	try {
		if (robot->state.edp.pid != -1) {
			//	printf("set_status inside\n");
			robot->ui_ecp_robot->the_robot->bird_hand_status_reply_data_request_port.set_request();
			robot->ui_ecp_robot->execute_motion();
			robot->ui_ecp_robot->the_robot->bird_hand_status_reply_data_request_port.get();

			if (robot->state.edp.is_synchronised)
				for (int i = 0; i < robot->number_of_servos; i++) {
					set_finger_status(i);
				}
			//init();
		}
	} // end try
	CATCH_SECTION_UI_PTR
}

void wgt_bird_hand_command::copy_command()
{
	if (robot->state.edp.pid != -1) {
		if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
		{
			ui->pushButton_execute->setDisabled(false);

			for (int i = 0; i < robot->number_of_servos; i++) {
				get_variant_finger_command(i);
				copy_finger_command(i);
			}

		} else {
			// Wygaszanie elementow przy niezsynchronizowanym robocie
			ui->pushButton_execute->setDisabled(true);
		}

	}
}

void wgt_bird_hand_command::on_pushButton_execute_clicked()
{
	get_command();
}

void wgt_bird_hand_command::get_variant_finger_command(int fingerId)
{
	QList <QAbstractButton*> buttons_in_group = buttonGroup_Vector[fingerId]->buttons();

	for (int i = 0; i < buttons_in_group.size(); i++) {
		if (buttons_in_group[i]->isChecked()) {
			switch (i)
			{
				case 0:
					joint_command[fingerId]->profile_type = lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION;
					break;
				case 1:
					joint_command[fingerId]->profile_type = lib::bird_hand::MACROSTEP_POSITION_INCREMENT;
					break;
				case 2:
					joint_command[fingerId]->profile_type = lib::bird_hand::SIGLE_STEP_POSTION_INCREMENT;
					break;
				default:
					break;
			}
			return;
		}
	}
}

void wgt_bird_hand_command::get_finger_command(int fingerId)
{
	joint_command[fingerId]->desired_position = desired_pos_spin_box[fingerId]->value();
	joint_command[fingerId]->desired_torque = doubleSpinBox_destor_Vector[fingerId]->value();
	joint_command[fingerId]->reciprocal_of_damping = doubleSpinBox_rdamp_Vector[fingerId]->value();

}

void wgt_bird_hand_command::set_finger_status(int fingerId)
{
	QList <QAbstractButton*> chboxes = checkboxButtonGroup_Vector[fingerId]->buttons();

	doubleSpinBox_curpos_Vector[fingerId]->setValue(joint_status[fingerId]->meassured_position);
	doubleSpinBox_curtor_Vector[fingerId]->setValue(joint_status[fingerId]->meassured_torque);
	doubleSpinBox_mcur_Vector[fingerId]->setValue(joint_status[fingerId]->measured_current);

	if (joint_status[fingerId]->lower_limit_of_absolute_position) {
		chboxes[0]->setChecked(true);
	} else {
		chboxes[0]->setChecked(false);
	}

	if (joint_status[fingerId]->lower_limit_of_absolute_value_of_desired_torque) {
		chboxes[1]->setChecked(true);
	} else {
		chboxes[1]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_position) {
		chboxes[2]->setChecked(true);
	} else {
		chboxes[2]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_computed_position_increment) {
		chboxes[3]->setChecked(true);
	} else {
		chboxes[3]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_desired_position_increment) {
		chboxes[4]->setChecked(true);
	} else {
		chboxes[4]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_desired_torque) {
		chboxes[5]->setChecked(true);
	} else {
		chboxes[5]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_absolute_value_of_meassured_torque) {
		chboxes[6]->setChecked(true);
	} else {
		chboxes[6]->setChecked(false);
	}

	if (joint_status[fingerId]->upper_limit_of_measured_current) {
		chboxes[7]->setChecked(true);
	} else {
		chboxes[7]->setChecked(false);
	}

}

void wgt_bird_hand_command::copy_finger_command(int fingerId)
{

	if (joint_command[fingerId]->profile_type == lib::bird_hand::MACROSTEP_ABSOLUTE_POSITION)
		desired_pos_spin_box[fingerId]->setValue(doubleSpinBox_curpos_Vector[fingerId]->value());
}

