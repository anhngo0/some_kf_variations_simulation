    #include "filter_lib.hpp"

    class PF {
        private:
            Eigen::MatrixXd Q_sqrt;
            Eigen::MatrixXd R_inv;
        
            Eigen::MatrixXd F;
            Eigen::MatrixXd particles;
            Eigen::VectorXd pweights;

            int N = 500;
            double dt;

            std::mt19937 gen;
            std::normal_distribution<double> dist;

        public:
            PF(double dt_):gen(std::random_device{}()),dist(0.0,1.0){
                dt = dt_;
                Eigen::MatrixXd Q = int_proc_noise_mat_Q_CV(dt);
                Eigen::LDLT<Eigen::MatrixXd> ldlt(Q);
                if(ldlt.info() != Eigen::Success)
                    throw std::runtime_error("init particles: Covariance matrix decomposition failed.");
                Q_sqrt = ldlt.matrixL();
                
                Eigen::MatrixXd R = int_meas_noise_mat_R_CV();
                Eigen::LDLT<Eigen::Matrix2d> ldlt2(R);
                if(ldlt2.info() != Eigen::Success)
                    throw std::runtime_error("PCRLB update: Measurement noise matrix decomposition failed.");
                R_inv = ldlt2.solve(Eigen::Matrix2d::Identity());
            }

            void init_particles(
                const Eigen::VectorXd& ukf_x,
                const Eigen::MatrixXd& ukf_P
            );

            void setF(const Eigen::MatrixXd& F_){F = F_;}
            void predict();
            void setParticlesNumber(int p_n){N = p_n;}

            void resample_if_needed();

            Eigen::VectorXd get_estimated_state();
            void update_weights_focus(const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos);

    };