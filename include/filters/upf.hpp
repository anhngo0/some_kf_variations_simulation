    
// #include "basic_filter.hpp"

// struct UPF_Particle{
//     UKF ukf;
//     Eigen::VectorXd prev_sample;   // x_{k-1}
//     Eigen::VectorXd sample;
//     double weight;

//     UPF_Particle(const UKF& filter, double w): ukf(filter),prev_sample(filter.getMean()), sample(filter.getMean()), weight(w){}
//     Eigen::VectorXd getEstMean() const{return sample * weight;}
// }; 

// class UPF {
//     private:
//         Eigen::MatrixXd Q_sqrt;
//         Eigen::MatrixXd R_inv;
//         std::vector<UPF_Particle> particles;
//         Eigen::MatrixXd F;

//         int N = 500;
//         double dt;

//         std::mt19937 gen;
//         std::normal_distribution<double> dist;

//         Eigen::LDLT<Eigen::MatrixXd> Q_ldlt;
//         double logDetQ;

//     public:
//         UPF(double dt_):gen(std::random_device{}()),dist(0.0,1.0), dt(dt_){
//             Eigen::MatrixXd Q = int_proc_noise_mat_Q_CV(dt);

//             Q_ldlt.compute(Q);

//             Eigen::LLT<Eigen::MatrixXd> llt(Q);
//             Q_sqrt = llt.matrixL();

//             logDetQ = 2.0 * Q_sqrt.diagonal().array().log().sum();

//             Eigen::MatrixXd R = int_meas_noise_mat_R_CV();

//             Eigen::LDLT<Eigen::MatrixXd> ldlt(R);
//             R_inv = ldlt.solve(Eigen::MatrixXd::Identity(R.rows(),R.cols()));
//         }

//         void init_particles(
//             const Eigen::VectorXd& ukf_x,
//             const Eigen::MatrixXd& ukf_P
//         );

//         void setF(const Eigen::MatrixXd& F_){F = F_;}
//         void predict();
//         void setParticlesNumber(int p_n){N = p_n;}

//         void resample_if_needed();
//         double logLikelihood(const Eigen::VectorXd& state, const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos);
//         double logTransition(const Eigen::VectorXd& x, const Eigen::VectorXd& prev);
     
//         Eigen::VectorXd get_estimated_state();
//         void update_weights_focus(const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos);
// };

    
#include "basic_filter.hpp"

struct UPF_Particle{
    Eigen::VectorXd prev_sample;   // x_{k-1}
    Eigen::VectorXd sample;
    Eigen::MatrixXd covariance;
    double weight;

    UPF_Particle(const Eigen::VectorXd& sample_,const Eigen::MatrixXd& cov, double w): 
    prev_sample(sample_), sample(sample_), covariance(cov), weight(w){}
    Eigen::VectorXd getEstMean() const{return sample * weight;}
}; 

class UPF {
    private:
        Eigen::MatrixXd Q_sqrt;
        Eigen::MatrixXd R_inv;
        std::vector<UPF_Particle> particles;
        Eigen::MatrixXd F;

        int N = 500;
        double dt;

        std::mt19937 gen;
        std::normal_distribution<double> dist;

        Eigen::LDLT<Eigen::MatrixXd> Q_ldlt;
        double logDetQ;

    public:
        UPF(double dt_):gen(std::random_device{}()),dist(0.0,1.0), dt(dt_){
            Eigen::MatrixXd Q = int_proc_noise_mat_Q_CV(dt);
            Q += 1e-6 * Eigen::MatrixXd::Identity(Q.rows(), Q.cols());
            Q_ldlt.compute(Q);

            Eigen::LLT<Eigen::MatrixXd> llt(Q);
            Q_sqrt = llt.matrixL();

            logDetQ = 2.0 * Q_sqrt.diagonal().array().log().sum();

            Eigen::MatrixXd R = int_meas_noise_mat_R_CV();

            Eigen::LDLT<Eigen::MatrixXd> ldlt(R);
            R_inv = ldlt.solve(Eigen::MatrixXd::Identity(R.rows(),R.cols()));
            // std::cout << "logDetQ = " << logDetQ << '\n';
            // std::cout << "Q determinant = " << Q.determinant() << '\n';
        }

        void init_particles(
            const Eigen::VectorXd& ukf_x,
            const Eigen::MatrixXd& ukf_P
        );

        void setF(const Eigen::MatrixXd& F_){F = F_;}
        void setParticlesNumber(int p_n){N = p_n;}

        void resample_if_needed();
        double logLikelihood(const Eigen::VectorXd& state, const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos);
        double logTransition(const Eigen::VectorXd& x, const Eigen::VectorXd& prev);
     
        Eigen::VectorXd get_estimated_state();
        void importance_sampling(const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos);
};