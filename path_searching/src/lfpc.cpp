#include <path_searching/lfpc.h>

using namespace std;
using namespace Eigen;

namespace cane_planner
{
    LFPC::LFPC()
    {

        // cycle init params
        x_0_ = 0.0;
        vx_0_ = 0.0;
        y_0_ = 0.0;
        vy_0_ = 0.0;
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;
        t_ = 0.0;
        // control param
        al_ = aw_ = theta_ = 0.0;
        b_ = 0.0;
        // pos init
        pos_foot_.Zero();
        left_foot_pos_.Zero();
        right_foot_pos_.Zero();
        COM_pos_.Zero();
        support_leg_pos_.Zero();
        // 初始高度
        COM_pos_(2) = h_;
        // step_path.
        step_path_.clear();
    }

    LFPC::~LFPC()
    {
        // pos init
        pos_foot_.Zero();
        left_foot_pos_.Zero();
        right_foot_pos_.Zero();
        COM_pos_.Zero();
        support_leg_pos_.Zero();
        step_path_.clear();
    }

    Vector4d LFPC::calculateXtVt(double t)
    {
        // in here,  iter_state  == [x_t,vx_t,y_t,vy_t]"
        Vector4d iter_state;
        // linear inverted pendulum motion low
        double tau = t / t_c_;
        // x
        iter_state(0) = x_0_ * cosh(tau) + t_c_ * vx_0_ * sinh(tau);
        iter_state(1) = x_0_ * sinh(tau) / t_c_ + vx_0_ * cosh(tau);
        // y
        iter_state(2) = y_0_ * cosh(tau) + t_c_ * vy_0_ * sinh(tau);
        iter_state(3) = y_0_ * sinh(tau) / t_c_ + vy_0_ * cosh(tau);
        return iter_state;
    }

    Vector4d LFPC::calculateFinalState()
    {
        Vector4d final_state;
        final_state = calculateXtVt(t_sup_);
        // std::cout << "final_state " << final_state.transpose() << std::endl;

        return final_state;
    }

    Vector2d LFPC::calculateLFPC(double vx, double vy)
    {
        Vector2d state_f;
        // Linear foot placement control
        if (support_leg_ == LEFT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) + aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) - aw_ * cos(theta_) + b_ * vy;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            state_f(0) = -al_ * cos(theta_) - aw_ * sin(theta_) + b_ * vx;
            state_f(1) = -al_ * sin(theta_) + aw_ * cos(theta_) + b_ * vy;
        }
        // std::cout << "LFPC set:" << state_f.transpose() << std::endl;
        return state_f;
    }

    void LFPC::updateOneDt()
    {
        t_ += delta_t_;
        Vector4d iter_state = calculateXtVt(t_);
        x_t_ = iter_state(0);
        vx_t_ = iter_state(1);
        y_t_ = iter_state(2);
        vy_t_ = iter_state(3);

        COM_pos_(0) = x_t_ + support_leg_pos_(0);
        COM_pos_(1) = y_t_ + support_leg_pos_(1);
    }

    void LFPC::updateNextFootLocation()
    {
        Vector4d final_state = calculateFinalState();
        Vector2d lfpc_set = calculateLFPC(final_state(1), final_state(3));
        if (support_leg_ == LEFT_LEG)
        {
            pos_foot_(0) = final_state(0) + left_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(2) + left_foot_pos_(1) + lfpc_set(1);
            right_foot_pos_ = pos_foot_;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            pos_foot_(0) = final_state(0) + right_foot_pos_(0) + lfpc_set(0);
            pos_foot_(1) = final_state(2) + right_foot_pos_(1) + lfpc_set(1);
            left_foot_pos_ = pos_foot_;
        }
        // std::cout << "update foot " << pos_foot_.transpose() << std::endl;

        return;
    }

    void LFPC::switchSupportLeg()
    {
        if (support_leg_ == LEFT_LEG)
        {
            support_leg_ = RIGHT_LEG;
            x_0_ = COM_pos_(0) - pos_foot_(0);
            y_0_ = COM_pos_(1) - pos_foot_(1);
            support_leg_pos_ = pos_foot_;
            // std::cout << "switch right_foot "
            //           << " x0: " << x_0_ << " y0: " << y_0_ << std::endl;
        }
        else if (support_leg_ == RIGHT_LEG)
        {
            support_leg_ = LEFT_LEG;
            x_0_ = COM_pos_(0) - pos_foot_(0);
            y_0_ = COM_pos_(1) - pos_foot_(1);
            support_leg_pos_ = pos_foot_;
            //     std::cout << "switch left_foot "
            //               << " x0: " << x_0_ << " y0: " << y_0_ << std::endl;
        }
        vx_0_ = vx_t_;
        vy_0_ = vy_t_;
        // std::cout << "vx and vy is: " << vx_0_ <<" and "<< vy_0_ << std::endl;
        t_ = 0;

        return;
    }

    void LFPC::updateOneStep()
    {
        step_path_.clear();
        int swing_data_len = int(t_sup_ / delta_t_);
        // std::cout << "this is step number " << step_num_ << std::endl;
        if (step_num_ == 0)
        {
            // first foot don"t need switch support leg
            updateNextFootLocation();

            // update motion com_pos into step_path_
            for (int i = 0; i < swing_data_len; i++)
            {
                updateOneDt();
                step_path_.push_back(COM_pos_);
            }
        }
        else
        {
            // update motion com_pos into step_path_
            for (int i = 0; i < swing_data_len; i++)
            {
                updateOneDt();
                step_path_.push_back(COM_pos_);
            }
            // update lfpc
            updateNextFootLocation();
        }
        step_num_ += 1;
    }
    void LFPC::initializeModel(ros::NodeHandle &nh)
    {
        std::cout << "\n----------initializeModel----------\n"
                  << std::endl;
        nh.param("lfpc/delta_t", delta_t_, 0.05);
        nh.param("lfpc/t_sup", t_sup_, 0.3);
        nh.param("lfpc/b_", b_, 0.3);
        nh.param("lfpc/h_", h_, 1.0);
        support_leg_ = LEFT_LEG;
        // calculate
        COM_pos_(2) = h_;
        t_c_ = sqrt(h_ / 10);
        std::cout << "tc is: " << t_c_ << std::endl;
    }
    void LFPC::reset(Vector4d init_state, Vector3d COM_init_pos,
                     Vector2d support_pos, char support_leg, int step_num)
    {
        step_num_ = step_num;
        t_ = 0;
        support_leg_ = support_leg;
        // step reinit
        x_t_ = 0.0;
        vx_t_ = 0.0;
        y_t_ = 0.0;
        vy_t_ = 0.0;
        // iter init
        x_0_ = init_state(0);
        vx_0_ = init_state(1);
        y_0_ = init_state(2);
        vy_0_ = init_state(3);
        // std::cout << "init state " << init_state.transpose() << std::endl;
        // pos reinit
        COM_pos_(0) = COM_init_pos(0);
        COM_pos_(1) = COM_init_pos(1);
        COM_pos_(2) = h_;

        // std::cout << "COM POS is " << COM_pos_.transpose() << std::endl;
        // and Others
        support_leg_pos_ << support_pos(0), support_pos(1), 0.0;
        if (support_leg_ == LEFT_LEG)
            left_foot_pos_ = support_leg_pos_;
        else if (support_leg_ == RIGHT_LEG)
            right_foot_pos_ = support_leg_pos_;

        pos_foot_ = support_leg_pos_;
        // clear path
        step_path_.clear();
    }
    void LFPC::SetCtrlParams(Vector3d input, double cur_theta)
    {
        al_ = input(0);
        aw_ = input(1);
        theta_ = cur_theta + input(2);
        if (theta_ > M_PI)
            theta_ -= M_PI;
        else if (theta_ < -M_PI)
            theta_ += M_PI;
        // std::cout << "al " << al_ << " aw_ " << aw_ << " theta_ " << theta_ << std::endl;
    }
    // 注意：updateOneStep()方法可以刷新path和pos_foot_，
    // 在updateOneStep后应该立即获取该轨迹和放置点
    Vector2d LFPC::getStepFootPosition()
    {
        Vector2d pos_foot_2d;
        pos_foot_2d << pos_foot_(0), pos_foot_(1);
        return pos_foot_2d;
    }
    Vector3d LFPC::getCOMPos()
    {
        return COM_pos_;
    }
    Vector4d LFPC::getNextIterState()
    {
        Vector4d state;
        state << x_0_, vx_0_, y_0_, vy_0_;
        return state;
    }
    char LFPC::getSupportFeet()
    {
        return support_leg_;
    }
    int LFPC::getStepNum()
    {
        return step_num_;
    }
    double LFPC::getTheta()
    {
        return theta_;
    }
    std::vector<Eigen::Vector3d> LFPC::getStepCOMPath()
    {
        return step_path_;
    }

} // namespace cane_planner
