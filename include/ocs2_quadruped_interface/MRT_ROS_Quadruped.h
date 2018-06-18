/*
 * MRT_ROS_Quadruped.h
 *
 *  Created on: May 25, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_QUADRUPED_H_
#define MRT_ROS_QUADRUPED_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <c_switched_model_interface/misc/CubicSpline.h>
#include <c_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include "c_switched_model_interface/foot_planner/FeetZDirectionPlanner.h"
#include <c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class MRT_ROS_Quadruped : public ocs2::MRT_ROS_Interface<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::logic_rules_t>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MRT_ROS_Quadruped<JOINT_COORD_SIZE>> Ptr;

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr 					quadruped_interface_ptr_t;
	typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
	typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
	typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

	enum {
		STATE_DIM = quadruped_interface_t::STATE_DIM,
		INPUT_DIM = quadruped_interface_t::INPUT_DIM,
		RBD_STATE_DIM = quadruped_interface_t::RBD_STATE_DIM
	};

	typedef ocs2::MRT_ROS_Interface<STATE_DIM, INPUT_DIM, typename quadruped_interface_t::logic_rules_t> BASE;
	typedef typename BASE::system_observation_t 	system_observation_t;
	typedef typename BASE::controller_t				controller_t;
	typedef typename BASE::controller_array_t		controller_array_t;
	typedef typename BASE::scalar_t					scalar_t;
	typedef typename BASE::scalar_array_t			scalar_array_t;
	typedef typename BASE::size_array_t				size_array_t;
	typedef typename BASE::state_vector_t			state_vector_t;
	typedef typename BASE::state_vector_array_t		state_vector_array_t;
	typedef typename BASE::input_vector_t			input_vector_t;
	typedef typename BASE::input_vector_array_t		input_vector_array_t;
	typedef typename BASE::input_state_matrix_t		input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t input_state_matrix_array_t;

	typedef FeetZDirectionPlanner<scalar_t,SplineCPG<scalar_t>>	feet_z_planner_t;
	typedef typename feet_z_planner_t::Ptr						feet_z_planner_ptr_t;

	// The base class for SplineCPG which is the return type of SwitchedModelPlannerLogicRules::getMotionPhaseLogics.
	typedef CPG_BASE<scalar_t>				cpg_t;
	typedef typename cpg_t::Ptr				cpg_ptr_t;
	typedef CubicSpline<scalar_t>			cubic_spline_t;
	typedef typename cubic_spline_t::Ptr	cubic_spline_ptr_t;

	typedef typename quadruped_interface_t::logic_rules_t 	logic_rules_t;
	typedef typename logic_rules_t::Ptr						logic_rules_ptr_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t,4>	vector_3d_array_t;

	/**
	 * Constructor
	 *
	 * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
	 * @param [in] robotName: The name's of the robot.
	 */
	MRT_ROS_Quadruped(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	/**
	 * Destructor
	 */
	virtual ~MRT_ROS_Quadruped() = default;

	/**
	 * Resets the class to its instantiate state.
	 */
	virtual void reset() override;

	/**
	 * Computes the optimized plan for the given time based on the latest received optimized trajectory message.
	 *
	 * @param [in] time: inquiry time.
	 * @param [out] o_feetPositionRef: Planned feet positions in the origin frame.
	 * @param [out] o_feetVelocityRef: Planned feet velocities in the origin frame.
	 * @param [out] o_feetAccelerationRef: Planned feet acceleration in the origin frame.
	 * @param [out] comPoseRef: Planned CoM pose in the origin frame.
	 * @param [out] comVelocityRef: Planned CoM velocity in the origin frame.
	 * @param [out] comAccelerationRef: Planned CoM acceleration in the origin frame.
	 * @param [out] stanceLegs: Planned stance legs.
	 */
	void computePlan(
			const scalar_t& time,
			vector_3d_array_t& o_feetPositionRef,
			vector_3d_array_t& o_feetVelocityRef,
			vector_3d_array_t& o_feetAccelerationRef,
			base_coordinate_t& o_comPoseRef,
			base_coordinate_t& o_comVelocityRef,
			base_coordinate_t& o_comAccelerationRef,
			contact_flag_t& stanceLegs);

	/**
	 * Updates the publisher and subscriber nodes. Using the input measurements it constructs the observation
	 * structure and publishes it using the base class method. It also checks calls the ros::spinOnce() and
	 * checks for the policy update using the base class method.
	 *
	 * @param [in] contactFlag: Current contact flag.
	 * @param [in] time: Current time.
	 * @param [in] rbdState: Current robot's RBD state
	 *
	 * @return True if the policy is updated.
	 */
	bool updateNodes(
			const contact_flag_t& contactFlag,
			const scalar_t& time,
			const rbd_state_vector_t& rbdState);

protected:
	/**
	 * This method can be sed to modify the feedforward policy on the buffer without inputting the
	 * main thread. Note that the variables that are on the buffer have the suffix Buffer. It is
	 * important if any new variables are added to the policy also obey this rule. These buffer
	 * variables can be later, in the customizedUpdatePolicy() method, swept to the in-use policy
	 * memory.
	 *
	 * @param [in] planInitObservationBuffer: The observation of the policy message on the buffer.
	 * @param mpcTimeTrajectoryBuffer: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectoryBuffer: The optimized state trajectory of the policy message on the buffer.
	 * @param mpcInputTrajectoryBuffer: The optimized input trajectory of the policy message on the buffer.
	 * @param eventTimesBuffer: The event times of the policy message on the buffer.
	 * @param subsystemsSequenceBuffer: The subsystems sequence of the policy message on the buffer.
	 */
	virtual void modifyBufferFeedforwardPolicy(
			const system_observation_t& planInitObservationBuffer,
			scalar_array_t& mpcTimeTrajectoryBuffer,
			state_vector_array_t& mpcStateTrajectoryBuffer,
			input_vector_array_t& mpcInputTrajectoryBuffer,
			scalar_array_t& eventTimesBuffer,
			size_array_t& subsystemsSequenceBuffer) override;

	/**
	 * The updatePolicy() method will call this method which allows the user to
	 * customize the in-use feedforward policy. Note that this method is already
	 * protected with a mutex which blocks the policy callback. Moreover, this method
	 * may be called in the main thread of the program. Thus, for efficiency and
	 * practical considerations you should avoid computationally expensive operations.
	 * For shuch operations you may want to use the modifyBufferFeedforwardPolicy()
	 * methods whihc runs on a separate thread which directly modifies the received
	 * policy messages on the data buffer.
	 *
	 * @param logicUpdated: Whether eventTimes or subsystemsSequence are updated form the last call.
	 * @param policyUpdated: Whether the policy is updated.
	 * @param mpcTimeTrajectory: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectory: The optimized state trajectory of the policy message on the buffer.
	 * @param mpcInputTrajectory: The optimized input trajectory of the policy message on the buffer.
	 * @param eventTimes: The event times of the policy.
	 * @param subsystemsSequence: The subsystems sequence of the policy.
	 */
	virtual void loadModifiedFeedforwardPolicy(
			bool& logicUpdated,
			bool& policyUpdated,
			scalar_array_t& mpcTimeTrajectory,
			state_vector_array_t& mpcStateTrajectory,
			input_vector_array_t& mpcInputTrajectory,
			scalar_array_t& eventTimes,
			size_array_t& subsystemsSequence) override;

	/**
	 * Updates feet trajectories.
	 *
	 * @param [in] eventTimes: Event times.
	 * @param [in] subsystemsSequence: Subsystems sequence.
	 * @param [in] touchdownTimeStock: An array of touch-down times.
	 * @param [in] touchdownStateStock: An array of feet positions at touch-down.
	 * @param [in] touchdownInputStock: An array of feet velocities at touch-down.
	 */
	void updateFeetTrajectories(
			const scalar_array_t& eventTimes,
			const size_array_t& subsystemsSequence,
			const scalar_array_t& touchdownTimeStock,
			const state_vector_array_t& touchdownStateStock,
			const input_vector_array_t& touchdownInputStock);

	/**
	 * Computes feet's position, velocity, and acting contact force in the origin frame.
	 *
	 * @param [in] state: state vector.
	 * @param [in] input: input vector.
	 * @param [out] o_feetPosition: Feet's position in the origin frame.
	 * @param [out] o_feetVelocity: Feet's velocity in the origin frame.
	 * @param [out] o_contactForces: Feet's acting contact force in the origin frame.
	 */
	void computeFeetState(
			const state_vector_t& state,
			const input_vector_t& input,
			vector_3d_array_t& o_feetPosition,
			vector_3d_array_t& o_feetVelocity,
			vector_3d_array_t& o_contactForces);

private:
	/*
	 * Variables
	 */
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;

	Model_Settings modelSettings_;

	std::array<const cpg_t*,4> 		feetZPlanPtr_;

	std::vector<std::array<cubic_spline_ptr_t,4>> feetXPlanPtrStock_;
	std::vector<std::array<cubic_spline_ptr_t,4>> feetYPlanPtrStock_;

	scalar_array_t			touchdownTimeStockBuffer_;
	state_vector_array_t 	touchdownStateStockBuffer_;
	input_vector_array_t 	touchdownInputStockBuffer_;

	state_vector_t stateRef_;
	input_vector_t inputRef_;
};

} // end of namespace switched_model

#include "implementation/MRT_ROS_Quadruped.h"


#endif /* MRT_ROS_QUADRUPED_H_ */
