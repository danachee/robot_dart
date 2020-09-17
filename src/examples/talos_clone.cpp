#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/stream.hpp>

#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

class TalosClonePool
{
public:
    static TalosClonePool *instance()
    {
        static TalosClonePool gdata;
        return &gdata;
    }

    TalosClonePool(const TalosClonePool &) = delete;
    void operator=(const TalosClonePool &) = delete;
    const std::string &model_filename() const { return _model_filename; }
    dart::dynamics::SkeletonPtr get_skeleton()
    {
        std::lock_guard<std::mutex> lock(_skeleton_mutex);
        for (size_t i = 0; i < _num_skeletons; i++)
        {
            if (_free[i])
            {
                _free[i] = false;
                return _skeletons[i];
            }
        }

        return nullptr;
    }

    void free_skeleton(const dart::dynamics::SkeletonPtr &skel)
    {
        std::lock_guard<std::mutex> lock(_skeleton_mutex);
        for (size_t i = 0; i < _num_skeletons; i++)
        {
            if (_skeletons[i] == skel)
            {
                _set_init_pos(skel);
                _free[i] = true;
                break;
            }
        }
    }

protected:
    std::vector<dart::dynamics::SkeletonPtr> _skeletons;
    std::vector<bool> _free;
    size_t _num_skeletons = 10; // set this number to your maximum number of concurrent skeletons (or slightly bigger to be sure! ;))
    std::mutex _skeleton_mutex;
    std::string _model_filename;

    TalosClonePool()
    {
        // load one skeleton from file and clone it _num_skeletons times
        // OR load _num_skeletons different skeletons from file
        // do not forget to fill _skeletons with your skeletons
        std::vector<std::pair<std::string, std::string>> packages = {{"talos_description", "talos/talos_description"}};
        std::cout << "Creating the pool of " << _num_skeletons << " robots..." << std::endl;
        for (size_t i = 0; i < _num_skeletons; ++i)
        {
            auto robot = std::make_shared<robot_dart::Robot>("talos/talos_fast.urdf", packages);
            _model_filename = robot->model_filename();
            _set_init_pos(robot->skeleton());
            _skeletons.push_back(robot->skeleton());
            std::cout << i << " ";
            std::cout.flush();
        }
        std::cout << "done" << std::endl;
        // Initialize all skeletons to free
        for (size_t i = 0; i < _num_skeletons; i++)
        {
            _free.push_back(true);
        }
    }

    void _set_init_pos(const dart::dynamics::SkeletonPtr &skel)
    {
        skel->resetPositions();
        skel->resetVelocities();
        skel->resetAccelerations();
        skel->clearExternalForces();

        // reset any altered properties (e.g., altered mass of a link) and set initial positions
    }
};

Eigen::VectorXd compute_velocities(std::shared_ptr<robot_dart::Robot> robot,
                                   const std::vector<std::string> &dofs,
                                   const Eigen::VectorXd &targetpos, double dt)
{
    Eigen::VectorXd q = robot->positions(dofs);
    Eigen::VectorXd vel = (targetpos - q) / dt;
    return vel;
}
int main()
{
    std::srand(std::time(NULL));

    // load the trajectory file (quick and dirty)
    int nrows = 10000, ncols = 38;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(nrows, ncols);
    {
        std::ifstream ifs("pos.dat.bz2", std::ios_base::in | std::ios_base::binary);
        boost::iostreams::filtering_stream<boost::iostreams::input> in;
        in.push(boost::iostreams::bzip2_decompressor());
        in.push(ifs);

        for (int row = 0; row < nrows; row++)
            for (int col = 0; col < ncols; col++)
            {
                double item = 0.0;
                in >> item;
                q(row, col) = item;
            }
    }
    for (int i = 0; i < 4; ++i) // loop over the clones
    {
        dart::dynamics::SkeletonPtr skel = nullptr;
        while (skel == nullptr)
        {
            skel = TalosClonePool::instance()->get_skeleton();
        }
        auto robot = std::make_shared<robot_dart::Robot>(skel);
        std::cout << "The model used is: [" << robot->model_filename() << "]" << std::endl;

        robot->set_position_enforced(true);

        robot->set_actuator_types("servo");

        double dt = 0.001;
        robot_dart::RobotDARTSimu simu(dt);
        simu.set_collision_detector("dart");
#ifdef GRAPHIC
        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(&simu);
        simu.set_graphics(graphics);
        graphics->look_at({0., 3.5, 2.}, {0., 0., 0.25});
        graphics->record_video("talos_dancing.mp4");
#endif
        simu.add_checkerboard_floor();
        simu.add_robot(robot);

        simu.set_control_freq(1000);
        std::vector<std::string> all_dofs = {"rootJoint_pos_x",
                                             "rootJoint_pos_y",
                                             "rootJoint_pos_z",
                                             "rootJoint_rot_x",
                                             "rootJoint_rot_y",
                                             "rootJoint_rot_z",
                                             "leg_left_1_joint",
                                             "leg_left_2_joint",
                                             "leg_left_3_joint",
                                             "leg_left_4_joint",
                                             "leg_left_5_joint",
                                             "leg_left_6_joint",
                                             "leg_right_1_joint",
                                             "leg_right_2_joint",
                                             "leg_right_3_joint",
                                             "leg_right_4_joint",
                                             "leg_right_5_joint",
                                             "leg_right_6_joint",
                                             "torso_1_joint",
                                             "torso_2_joint",
                                             "arm_left_1_joint",
                                             "arm_left_2_joint",
                                             "arm_left_3_joint",
                                             "arm_left_4_joint",
                                             "arm_left_5_joint",
                                             "arm_left_6_joint",
                                             "arm_left_7_joint",
                                             "gripper_left_joint",
                                             "arm_right_1_joint",
                                             "arm_right_2_joint",
                                             "arm_right_3_joint",
                                             "arm_right_4_joint",
                                             "arm_right_5_joint",
                                             "arm_right_6_joint",
                                             "arm_right_7_joint",
                                             "gripper_right_joint",
                                             "head_1_joint",
                                             "head_2_joint"};

        int ncontrollables = all_dofs.size() - 6;
        std::vector<std::string> controllable_dofs(ncontrollables);
        for (int j = 0; j < controllable_dofs.size(); ++j)
            controllable_dofs[j] = all_dofs[j + 6];

        auto start = std::chrono::steady_clock::now();
        int k = 1;

        robot->set_positions(q.row(0), all_dofs);

        // a few steps to be sure that we are stable
        for (int i = 0; i < 2000; ++i)
            simu.step_world();

        while (simu.scheduler().next_time() < 15. && !simu.graphics()->done())
        {
            auto q_k = q.row(k % q.rows());
            auto cmd = compute_velocities(robot, all_dofs, q_k, dt);
            robot->set_commands(cmd.tail(ncontrollables), controllable_dofs);
            simu.step_world();
            ++k;
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "benchmark time: " << elapsed_seconds.count() << "s\n";

        TalosClonePool::instance()->free_skeleton(skel);
    }
    return 0;
}
