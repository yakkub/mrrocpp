#include "wgt_input_integer.h"
#include "../interface.h"
#include "../ui_ecp.h"
#include <QHideEvent>

wgt_input_integer::wgt_input_integer(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
	wgt_base("Input Integer Dialog", _interface, parent), ui(new Ui::wgt_input_integerClass)
{
	ui->setupUi(this);

}

wgt_input_integer::~wgt_input_integer()
{
	delete ui;
}

void wgt_input_integer::hideEvent(QHideEvent *event)
{
	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
		interface.ui_ecp_obj->ui_rep.integer_number = 0;

	}
	interface.ui_ecp_obj->synchroniser.command();
	event->accept();
}

void wgt_input_integer::on_pushButton_ok_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::ANSWER_YES;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.integer_number = ui->spinBox_input->value();
	my_close();
}

void wgt_input_integer::on_pushButton_cancel_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	interface.ui_ecp_obj->ui_rep.integer_number = 0;
	my_close();
}

void wgt_input_integer::my_open(bool set_on_top)
{
	ui->label_message->setText(interface.ui_ecp_obj->ecp_to_ui_msg.string);
	wgt_base::my_open(set_on_top);
}

