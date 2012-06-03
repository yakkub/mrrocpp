#include "wgt_choose_option.h"
#include "../interface.h"
#include "../ui_ecp.h"
#include <QHideEvent>

wgt_choose_option::wgt_choose_option(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Choose Option Dialog", _interface, parent), ui(new Ui::wgt_choose_optionClass)
{
	ui->setupUi(this);

}

wgt_choose_option::~wgt_choose_option()
{
	delete ui;
}

void wgt_choose_option::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;

	}
	interface.ui_ecp_obj->synchroniser.command();
	event->accept();
}

void wgt_choose_option::on_pushButton_1_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_ONE;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_choose_option::on_pushButton_2_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_TWO;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_choose_option::on_pushButton_3_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_THREE;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_choose_option::on_pushButton_4_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::OPTION_FOUR;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_choose_option::on_pushButton_cancel_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_NO;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	my_close();
}

void wgt_choose_option::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);

	//ui->alpha_input->setValue(interface.ui_ecp_obj->ecp_to_ui_msg.PkmItem.alpha());

	switch (interface.ui_ecp_obj->ecp_to_ui_msg.nr_of_options)
	{
		case 2:
			ui->pushButton_3->hide();
			ui->pushButton_4->hide();
			break;
		case 3:
			ui->pushButton_3->show();
			ui->pushButton_4->hide();
			break;
		case 4:
			ui->pushButton_3->show();
			ui->pushButton_4->show();
			break;
		default:
			break;
	}

	wgt_base::my_open(set_on_top);
}
