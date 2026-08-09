// #include "../../include/filters/upf.hpp"


// void UPF::init_particles(
//     const Eigen::VectorXd& init_x,
//     const Eigen::MatrixXd& init_P
// ){
//     particles.clear();
//     particles.reserve(N);

//     Eigen::LDLT<Eigen::MatrixXd> ldlt(init_P);
//     if(ldlt.info() != Eigen::Success)
//         throw std::runtime_error("init particles: Covariance matrix decomposition failed.");
//     Eigen::MatrixXd P_sqrt = ldlt.matrixL();

//     /*zi ~ N(0, I)*/
//     Eigen::MatrixXd Z(init_x.size(), N);

//     for(int j=0;j<N;j++)
//         for(int i=0;i<init_x.size();i++)
//             Z(i,j)=dist(gen);

//     Eigen::MatrixXd X = init_x.replicate(1,N)+P_sqrt*Z;
    
//     for(int i=0;i<N;i++){
//         UKF ukf(X.col(i),init_P,dt);
//         ukf.setF(F);

//         particles.emplace_back(ukf, 1.0 / N);
//     }
// };

// Eigen::VectorXd UPF::get_estimated_state(){
//     int state_dim = particles[0].ukf.getMean().size();
//     Eigen::VectorXd res = Eigen::VectorXd::Zero(state_dim);
//     for(const auto& p : particles){
//         res += p.getEstMean();
//     }
//     return res; 
// };

// void UPF::resample_if_needed()
// {
//     //---------------------------------
//     // Compute ESS
//     //---------------------------------
//     double sum_sq = 0.0;
//     for(const auto& p : particles)
//         sum_sq += p.weight * p.weight;

//     double ess = 1.0 / sum_sq;

//     if(ess >= 0.5 * N)
//         return;

//     //---------------------------------
//     // Systematic resampling
//     //---------------------------------
//     std::vector<UPF_Particle> new_particles;
//     new_particles.reserve(N);

//     std::uniform_real_distribution<double> uni(0.0, 1.0 / N);

//     double r = uni(gen);
//     double c = particles[0].weight;
//     int i = 0;

//     for(int m = 0; m < N; ++m)
//     {
//         double U = r + (double)m / N;

//         while(U > c && i < N - 1)
//         {
//             ++i;
//             c += particles[i].weight;
//         }

//         new_particles.push_back(particles[i]);      // copy whole particle
//         new_particles.back().weight = 1.0 / N;      // reset weight
//     }

//     particles.swap(new_particles);
// }

// void UPF::predict(){
    
//     for(auto& p : particles){
//         p.prev_sample = p.sample;
//         p.ukf.predict();
//     }
// };

// /*p(z_k | x_k)*/
// double UPF::logLikelihood(
//     const Eigen::VectorXd& state,
//     const Eigen::VectorXd& meas,
//     const Eigen::VectorXd& obs_pos)
// {
//     Eigen::VectorXd z_pred = nonlinear_meas_func(state, obs_pos);
//     Eigen::Vector2d innov = meas - z_pred;

//     normalize_angle(innov(0));

//     double d2 = innov.transpose() * R_inv * innov;

//     return -0.5 * d2;
// }

// /*q(x)=N(μ,P)*/
// double logProposal(
//     const Eigen::VectorXd& x,
//     const Eigen::VectorXd& mean,
//     const Eigen::MatrixXd& cov
// ){
//     Eigen::LLT<Eigen::MatrixXd> llt(cov);
//     Eigen::MatrixXd L = llt.matrixL();

//     double logdet =2.0*L.diagonal().array().log().sum();
//     Eigen::VectorXd diff=x-mean;

//     Eigen::VectorXd y = llt.solve(diff);

//     double quad= diff.dot(y);

//     int n=x.size();

//     return -0.5* (quad+logdet+n*std::log(2.0*M_PI));
// }

// double UPF::logTransition(const Eigen::VectorXd& x,const Eigen::VectorXd& prev){
//     Eigen::VectorXd mean= F * prev;
//     Eigen::VectorXd diff= x - mean;

//     Eigen::VectorXd y= Q_ldlt.solve(diff);

//     double quad= diff.dot(y);

//     double logdet= logDetQ;
//     int n=x.size();

//     return -0.5*(quad+logdet+n*std::log(2*M_PI));
// }



// void UPF::update_weights_focus(
//     const Eigen::VectorXd& meas,
//     const Eigen::VectorXd& obs_pos)
// {
//     //------------------------------------
//     // UKF update
//     //------------------------------------

//     std::vector<double> log_weights(N);
//     #pragma omp parallel 
//     {
//         std::mt19937 local_gen(std::random_device{}()+omp_get_thread_num());
//         std::normal_distribution<double> local_dist(0.0,1.0);

//         #pragma omp for schedule(static)
//         for(int i = 0; i < N; ++i){
//             auto& p = particles[i];
//             p.ukf.update(meas,obs_pos);

//             /*sample \hat{x}_t ~ N(\hat{x}_t, P_t)*/
//             Eigen::VectorXd mu = p.ukf.getMean();
//             Eigen::MatrixXd P = p.ukf.getCovariance();
//             Eigen::LLT<Eigen::MatrixXd> llt(P);
//             Eigen::MatrixXd L = llt.matrixL();
//             Eigen::VectorXd noise(mu.size());

//             for(int k=0;k<mu.size();k++)
//                 noise(k)=local_dist(local_gen);

//             p.sample =mu + L*noise;
//             /*evaluate the imp ortance weights up to a normalizing constant:*/
//             /*w_t^(i) \alpha p(y_t | x_t) * p(x_t | x_{t-1}) / q(x_t|x_{0:t-1}, y_{1:t})*/
//             double l1 = logLikelihood(p.sample,meas,obs_pos);
//             double l2 = logTransition(p.sample,p.prev_sample);
//             double l3 = logProposal(p.sample,mu,P);
//             log_weights[i] = std::log(p.weight + 1e-300) + l1 + l2 - l3;
//             std::cout
//                 << l1 << " "
//                 << l2 << " "
//                 << l3 << std::endl;
//         }
//     }

//     //------------------------------------
//     // normalize
//     //------------------------------------
    
//     double max_log = *std::max_element(log_weights.begin(),log_weights.end());
//     double sum = 0.0;

//     for(int i=0;i<N;i++){
//         particles[i].weight = std::exp(log_weights[i]-max_log);
//         sum += particles[i].weight;
//     }

//     for(auto& p:particles)
//         p.weight /= (sum + 1e-300);
// }

#include "../../include/filters/upf.hpp"


void UPF::init_particles(
    const Eigen::VectorXd& init_x,
    const Eigen::MatrixXd& init_P
){
    particles.clear();
    particles.reserve(N);

    Eigen::LDLT<Eigen::MatrixXd> ldlt(init_P);
    if(ldlt.info() != Eigen::Success)
        throw std::runtime_error("init particles: Covariance matrix decomposition failed.");
    Eigen::MatrixXd P_sqrt = ldlt.matrixL();

    /*zi ~ N(0, I)*/
    Eigen::MatrixXd Z(init_x.size(), N);

    for(int j=0;j<N;j++)
        for(int i=0;i<init_x.size();i++)
            Z(i,j)=dist(gen);

    Eigen::MatrixXd X = init_x.replicate(1,N)+P_sqrt*Z;
    
    for(int i=0;i<N;i++){
        particles.emplace_back(X.col(i),init_P, 1.0 / N);
    }
};

Eigen::VectorXd UPF::get_estimated_state(){
    int state_dim = particles[0].sample.size();
    Eigen::VectorXd res = Eigen::VectorXd::Zero(state_dim);
    for(const auto& p : particles){
        res += p.getEstMean();
    }
    return res; 
};

void UPF::resample_if_needed()
{
    //---------------------------------
    // Compute ESS
    //---------------------------------
    double sum_sq = 0.0;
    for(const auto& p : particles)
        sum_sq += p.weight * p.weight;

    double ess = 1.0 / sum_sq;
    // std::cout << "| ESS is " << ess << "\n";
    if(ess >= 0.5 * N)
        return;

    //---------------------------------
    // Systematic resampling
    //---------------------------------
    std::vector<UPF_Particle> new_particles;
    new_particles.reserve(N);

    std::uniform_real_distribution<double> uni(0.0, 1.0 / N);

    double r = uni(gen);
    double c = particles[0].weight;
    int i = 0;

    for(int m = 0; m < N; ++m)
    {
        double U = r + (double)m / N;

        while(U > c && i < N - 1)
        {
            ++i;
            c += particles[i].weight;
        }

        new_particles.push_back(particles[i]);      // copy whole particle
        new_particles.back().weight = 1.0 / N;      // reset weight
    }

    particles.swap(new_particles);
}

/*p(z_k | x_k)*/
double UPF::logLikelihood(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& meas,
    const Eigen::VectorXd& obs_pos)
{
    Eigen::VectorXd z_pred = nonlinear_meas_func(state, obs_pos);
    Eigen::Vector2d innov = meas - z_pred;

    normalize_angle(innov(0));

    double d2 = innov.transpose() * R_inv * innov;

    return -0.5 * d2;
}

/*q(x)=N(μ,P)*/
double logProposal(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& mean,
    const Eigen::MatrixXd& cov
){
    Eigen::LLT<Eigen::MatrixXd> llt(cov);
    Eigen::MatrixXd L = llt.matrixL();

    double logdet =2.0*L.diagonal().array().log().sum();
    Eigen::VectorXd diff=x-mean;

    Eigen::VectorXd y = llt.solve(diff);

    double quad= diff.dot(y);

    int n=x.size();

    return -0.5* (quad+logdet+n*std::log(2.0*M_PI));
}

double UPF::logTransition(const Eigen::VectorXd& x,const Eigen::VectorXd& prev){
    Eigen::VectorXd mean= F * prev;
    Eigen::VectorXd diff= x - mean;

    Eigen::VectorXd y= Q_ldlt.solve(diff);

    double quad= diff.dot(y);

    double logdet= logDetQ;
    int n=x.size();

    return -0.5*(quad+logdet+n*std::log(2*M_PI));
}



void UPF::importance_sampling(
    const Eigen::VectorXd& meas,
    const Eigen::VectorXd& obs_pos)
{
    //------------------------------------
    // UKF update
    //------------------------------------

    std::vector<double> log_weights(N);
    #pragma omp parallel 
    {
        std::mt19937 local_gen(std::random_device{}()+omp_get_thread_num());
        std::normal_distribution<double> local_dist(0.0,1.0);

        #pragma omp for schedule(static)
        for(int i = 0; i < N; ++i){
            auto& p = particles[i];
            UKF ukf(p.sample, p.covariance, dt);
            ukf.setF(F);
            // ukf.setAlpha(0.8);

            p.prev_sample = p.sample;
            ukf.predict();
            ukf.update(meas,obs_pos);

            /*sample \hat{x}_t ~ N(\hat{x}_t, P_t)*/
            Eigen::VectorXd mu = ukf.getMean();
            Eigen::MatrixXd P = ukf.getCovariance();
            Eigen::LLT<Eigen::MatrixXd> llt(P);
            Eigen::MatrixXd L = llt.matrixL();
            Eigen::VectorXd noise(mu.size());

            for(int k=0;k<mu.size();k++)
                noise(k)=local_dist(local_gen);

            p.sample =mu + L*noise;
            p.covariance = P;
            /*evaluate the imp ortance weights up to a normalizing constant:*/
            /*w_t^(i) \alpha p(y_t | x_t) * p(x_t | x_{t-1}) / q(x_t|x_{0:t-1}, y_{1:t})*/
            double l1 = logLikelihood(p.sample,meas,obs_pos);
            double l2 = logTransition(p.sample,p.prev_sample);
            double l3 = logProposal(p.sample,mu,P);
            log_weights[i] = std::log(p.weight + 1e-300) + l1 + l2 - l3;
            // log_weights[i] = std::log(p.weight + 1e-300);

            // if(i == 0)
                // {
                //     Eigen::VectorXd priorMean = F * p.prev_sample;

                //     std::cout
                //         << "||mu - F*x_prev|| = "
                //         << (mu - priorMean).norm()
                //         << '\n';
                // }
            // {
            //     std::cout
            //     << "l1 = " << l1
            //     << "  l2 = " << l2
            //     << "  l3 = " << l3
            //     << std::endl;
            // }
        }
        
    }

    //------------------------------------  
    // normalize
    //------------------------------------
    
    double max_log = *std::max_element(log_weights.begin(),log_weights.end());
    double sum = 0.0;

    for(int i=0;i<N;i++){
        particles[i].weight = std::exp(log_weights[i]-max_log);
        sum += particles[i].weight;
    }

    for(auto& p:particles)
        p.weight /= (sum + 1e-300);
}