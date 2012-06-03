/**
 * @file
 * @brief Contains declarations of the methods of newsmooth class.
 * @author rtulwin
 * @ingroup generators
 */

#if !defined(_ECP_GEN_NEWSMOOTH_H)
# define _ECP_GEN_NEWSMOOTH_H

#include "generator/ecp/ecp_g_multiple_position.h"
#include "base/lib/trajectory_pose/bang_bang_trajectory_pose.h"
#include "base/lib/datastr.h"
#include "generator/lib/velocity_profile_calculator/bang_bang_profile.h"
#include "generator/lib/trajectory_interpolator/bang_bang_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief Smooth trajectory generator which has an ability to calculate every trajectory (posiada moce super krowy).
 *
 * Usage:
 * Load one or more of trajectory poses using one of the load methods. Velocities and accelerations are set automatically, however they can be
 * also set by the appropriate load methods. Call %calculate_interpolate() method.
 * If it returns true generator is ready to communicate with the robot. Call the %Move() method. The generator resets itself automatically after
 * successful termination of the assumed trajectory, however it is safe to call the %reset() method before the next use of the generator.
 *
 * @author rtulwin
 * @ingroup generators
 */
class newsmooth : public multiple_position<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
ecp::common::generator::velocity_profile_calculator::bang_bang_profile> {

	private:
		/**
		 * Creates the vectors containning the information about the maximal and typical velocities and accelerations for each representation.
		 * @param axes_num actual number of axes
		 */
		void create_velocity_vectors(int axes_num);
		/**
		 * Calculates trajectory.
		 * @return true if calculation was successful.
		 */
		bool calculate();
		/**
		 * Loads trajectory pose.
		 * @return true if the addition was successful
		 */
		bool load_trajectory_pose(const std::vector<double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const std::vector<double> & v, const std::vector<double> & a, const std::vector<double> & v_max, const std::vector<double> & a_max);
		/**
		 * Method used to print list of positions.
		 */
		void print_pose_vector();
		/**
		 * Prints single pose.
		 */
		void print_pose(std::vector<ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it);
                /**
                 * Performs basic optimization of the motion by setting new values of maximal velocity and maximal acceleration separately on each trajectory segment.
                 * Optimization is based on minimizing the energy cost.
                 * @return true if optimization finished
                 */
                bool optimize_energy_cost(std::vector<double> max_current, std::vector<double> max_current_change, std::vector<double> max_velocity, std::vector<double> max_acceleration, double stopCondition);

	public:
		/**
		 * Constructor.
		 */
		newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num);
		/**
		 * Destructor.
		 */
		virtual ~newsmooth();
		/**
		 * Loads a single trajectory pose described in joint coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in joint coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_joint_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in motor coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_motor_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in euler zyz coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_euler_zyz_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (absolute motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_absolute_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads a single trajectory pose described in angle axis coordinates (relative motion) to the list. Maximal velocities and accelerations are set automatically.
		 * @param coordinates desired position
		 * @return true if the addition was successful
		 */
		bool load_relative_angle_axis_trajectory_pose(const std::vector<double> & coordinates);
		/**
		 * Loads the whole trajectory chain (possibly more than one position) stored in a text file.
		 * @param file_name name of the file with the trajectory
		 */
		bool load_trajectory_from_file(const char* file_name);
		/**
		 * Loads the whole absolute trajectory pose.
		 * @param trajectory to load
		 */
		bool load_absolute_pose(ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose & trajectory_pose);
		/**
		 * Loads the whole relative trajectory pose.
		 * @param trajectory to load
		 */
		bool load_relative_pose(ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose & trajectory_pose);
                /**
                 * Performs basic optimization of the motion by setting new values of maximal velocity and maximal acceleration separately on each trajectory segment.
                 * Optimization is based on minimizing the energy cost.
                 */
                void optimize_energy_cost(std::vector<double> startPos, std::vector<double> max_current, std::vector<double> max_current_change, std::vector<double> max_velocity, std::vector<double> max_acceleration, double stop_condition, boost::shared_ptr <newsmooth> sgenstart, const char* file_name);
                /**
                 * Performs basic optimization of the motion of irp6-postument robot by setting new values of maximal velocity and maximal acceleration separately on each trajectory segment.
                 * Optimization is based on minimizing the energy cost.
                 */
                void optimize_energy_cost_postument(boost::shared_ptr <newsmooth> sgenstart, const char *file_name, std::vector<double> start_pos, double stop_condition);
                /**
                 * Performs basic optimization of the motion of irp6-track robot by setting new values of maximal velocity and maximal acceleration separately on each trajectory segment.
                 * Optimization is based on minimizing the energy cost.
                 */
                void optimize_energy_cost_track(boost::shared_ptr <newsmooth> sgenstart, const char *file_name, std::vector<double> start_pos, double stop_condition);
};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
