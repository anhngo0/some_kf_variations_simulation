#include "../../include/filters/pf.hpp"

 void PF::init_particles(
    const Eigen::VectorXd& init_x,
    const Eigen::MatrixXd& init_P
){
    particles.resize(init_x.size(), N);
    pweights = Eigen::VectorXd::Constant(N, 1.0 / N);

    Eigen::LDLT<Eigen::MatrixXd> ldlt(init_P);
    if(ldlt.info() != Eigen::Success)
        throw std::runtime_error("init particles: Covariance matrix decomposition failed.");
    Eigen::MatrixXd P_sqrt = ldlt.matrixL();

    /*zi ~ N(0, I)*/
    Eigen::MatrixXd Z(init_x.size(), N);

    for(int j=0;j<N;j++)
        for(int i=0;i<init_x.size();i++)
            Z(i,j)=dist(gen);

    particles = init_x.replicate(1,N) + P_sqrt*Z;
};

void PF::predict(){
    int state_dim = particles.rows();

    /*zi ~ N(0, I)*/
    Eigen::MatrixXd Z(state_dim, N) ;

    for(int j=0;j<N;j++)
        for(int i=0;i<state_dim;i++)
            Z(i,j)=dist(gen);

    particles = F*particles +  Q_sqrt * Z;
};

void PF::resample_if_needed()
{
    double ess = 1.0 / pweights.array().square().sum();
    // std::cout << " ESS is " << ess << std::endl;
    if(ess >= 0.5 * N)
        return;

    Eigen::MatrixXd new_particles(particles.rows(),N);

    //---------------------------------
    // Systematic resampling
    //---------------------------------

    std::uniform_real_distribution<double> uni(0.0,1.0/N);

    double r = uni(gen);
    double c = pweights(0);

    int i = 0;

    for(int m=0;m<N;m++){
        double U = r + m/(double)N;
        while(U > c && i < N-1){
            i++;
            c += pweights(i);
        }

        new_particles.col(m) =particles.col(i);
    }

    particles.swap(new_particles);
    pweights.setConstant(1.0/N); /*reset state*/
}

Eigen::VectorXd PF::get_estimated_state(){
    return particles * pweights; 
};

void PF::update_weights_focus(const Eigen::VectorXd& meas, const Eigen::VectorXd& obs_pos){

    Eigen::VectorXd log_weights(N);

    #pragma omp parallel for schedule(static)
    for(int i=0;i<N;i++){
        Eigen::VectorXd z_pred = nonlinear_meas_func(particles.col(i), obs_pos);

        Eigen::Vector2d innov = meas - z_pred;

        normalize_angle(innov(0));

        double d2 = innov.transpose() * R_inv * innov;

        log_weights(i) = std::log(pweights(i)+1e-12) -0.5*d2;
    }

    //-----------------------------------
    // log-sum-exp normalization
    //-----------------------------------

    double max_log = log_weights.maxCoeff();

    pweights =(log_weights.array()-max_log).exp();

    pweights /= pweights.sum();

};