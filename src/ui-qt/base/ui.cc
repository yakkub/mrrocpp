/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "ui.h"
#include "interface.h"

namespace mrrocpp {
namespace ui {
namespace common {

busy_flagger::busy_flagger(busy_flag & _flag) :
		flag(_flag)
{
	flag.increment();
}

busy_flagger::~busy_flagger()
{
	flag.decrement();
}

busy_flag::busy_flag() :
		counter(0)
{
}

void busy_flag::increment(void)
{
	boost::mutex::scoped_lock lock(m_mutex);
	counter++;
}

void busy_flag::decrement(void)
{
	boost::mutex::scoped_lock lock(m_mutex);
	counter--;
}

bool busy_flag::is_busy() const
{
	//	boost::mutex::scoped_lock lock(m_mutex);
	return (counter);
}

function_execution_buffer::function_execution_buffer(Interface& _interface) :
		interface(_interface), has_command(false)
{
}

void function_execution_buffer::command(command_function_t _com_fun)
{
	boost::unique_lock <boost::mutex> lock(mtx);

	// assign command for execution
	com_fun = _com_fun;
	has_command = true;

	cond.notify_one();

	return;
}

int function_execution_buffer::wait_and_execute()
{
	command_function_t popped_command;

	{
		boost::unique_lock <boost::mutex> lock(mtx);

		while (!has_command) {
			cond.wait(lock);
		}

		has_command = false;
		popped_command = com_fun;
	}

	busy_flagger flagger(interface.communication_flag);

	// interface.set_ui_state_notification(UI_N_BUSY);

	return popped_command();
}

void feb_thread::operator()()
{

	feb.interface.mask_signals_for_thread();

	while (true) {
		feb.wait_and_execute();
	}
}

feb_thread::feb_thread(function_execution_buffer & _feb) :
		feb(_feb)
{
	thread_id = boost::thread(boost::bind(&feb_thread::operator(), this));
}

feb_thread::~feb_thread()
{
	thread_id.interrupt();
	thread_id.join(); // join it
}

}
} //namespace ui
} //namespace mrrocpp
