#include "../include/simulation.hpp"

std::vector<GT_State_Per_Frame> create_observer_traj(){
    Trajectory observer;
    GT_State_Per_Frame observer_initial_state;
    observer_initial_state.frame = 0.0;
    observer_initial_state.vel << 0.0 , 0.03 * 1e4 , 0.0; //initial velocity 0.03m/s - y axis
    std::unique_ptr<MotionSegment> obs_CV1 = std::make_unique<CV>(0,15,observer_initial_state);
    std::unique_ptr<MotionSegment> obs_CT1 = std::make_unique<CT>(15,31, -ROTATE_ANGLE_VEL);
    std::unique_ptr<MotionSegment> obs_CV2 = std::make_unique<CV>(31,43);
    std::unique_ptr<MotionSegment> obs_CT2 = std::make_unique<CT>(43,75, ROTATE_ANGLE_VEL);
    std::unique_ptr<MotionSegment> obs_CV3 = std::make_unique<CV>(75,86);
    std::unique_ptr<MotionSegment> obs_CT3 = std::make_unique<CT>(86,102, -ROTATE_ANGLE_VEL);
    std::unique_ptr<MotionSegment> obs_CV4 = std::make_unique<CV>(102,MAX_FRAME);
    observer.addSegment(std::move(obs_CV1));
    observer.addSegment(std::move(obs_CT1));
    observer.addSegment(std::move(obs_CV2));
    observer.addSegment(std::move(obs_CT2));
    observer.addSegment(std::move(obs_CV3));
    observer.addSegment(std::move(obs_CT3));
    observer.addSegment(std::move(obs_CV4));

    observer.generate(FRAME_STEP);
    return observer.getGroundTruth();
}

std::vector<GT_State_Per_Frame> create_target_traj(){
    Trajectory target_traj;
    GT_State_Per_Frame target_init_state;
    target_init_state.frame = 0.0;
    target_init_state.pos << 97580.736 , 97580.736 , 9 * 1e3;
    target_init_state.vel << -110.011 , -110.011 , -10.0;
    target_init_state.acc << -5, -5, -2;

    std::unique_ptr<MotionSegment> CA1 = std::make_unique<CA>(0, 20, target_init_state);
    std::unique_ptr<MotionSegment> CV1 = std::make_unique<CV>(20, MAX_FRAME-10);
    std::unique_ptr<MotionSegment> CA2 = std::make_unique<CV>(MAX_FRAME -10, MAX_FRAME);
    
    target_traj.addSegment(std::move(CA1));
    target_traj.addSegment(std::move(CV1));
    target_traj.addSegment(std::move(CA2));
    target_traj.generate(FRAME_STEP);

    return target_traj.getGroundTruth();
}

void saveTrajectoryToJson(
    const std::vector<GT_State_Per_Frame>& traj,
    const std::string& filename
)
{
    json j;
    j["trajectory"] = json::array();

    for (const auto& state : traj)
    {
        j["trajectory"].push_back({
            {"frame", state.frame},
            {"position", {
                state.pos.x(),
                state.pos.y(),
                state.pos.z()
            }}
        });
    }

    std::ofstream file(filename);

    if (!file.is_open())
        throw std::runtime_error("Cannot open file: " + filename);

    file << j.dump(4);   // indent = 4 spaces
}

void saveRmsePerFrameInJson(
    const std::vector<RMSE_per_Frame>& rmse_list,
    const std::string& filename1, 
    const std::string& filename2
){
    {
        std::ofstream file(filename1);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + filename1);

        json j;
        j["rmse_pos"] = json::array();
        for(const auto& r: rmse_list){
            j["rmse_pos"].push_back({
                {"frame", r.frame},
                {"rmse", r.pos}
            });
        }
        file << j.dump(4); 
    }

    {
        std::ofstream file(filename2);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + filename2);

        json j;
        j["rmse_vel"] = json::array();

        for(const auto& r: rmse_list){
            j["rmse_vel"].push_back({
                {"frame", r.frame},
                {"rmse", r.vel}
            });
        }
        file << j.dump(4); 
    }
}

double normal(double mean, double stddev) {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
}

void simulate(){

    /*===================== Create Observer and Target Trajectory ======================*/
    
    std::vector<GT_State_Per_Frame> observer_traj = create_observer_traj();
    saveTrajectoryToJson(observer_traj, "json/observer_traj.json"); /*for visualize*/

    std::vector<GT_State_Per_Frame> target_traj = create_target_traj();
    saveTrajectoryToJson(target_traj, "json/target_traj.json"); /*for visualize*/

    /*==================================================================================*/
    /*====================== Monte Carlo sample filter process =========================*/
    /*==================================================================================*/
    std::vector<RMSE_per_Frame> rmse_per_frame_list_ekf;
    std::vector<RMSE_per_Frame> rmse_per_frame_list_ukf;
    std::vector<RMSE_per_Frame> rmse_per_frame_list_pf;
    std::vector<RMSE_per_Frame> rmse_per_frame_list_pcrlb;
    std::vector<RMSE_per_Frame> rmse_per_frame_list_upf;

    /*MONTE CARLO SAMPLE*/
    for(int s = 0; s < MONTE_CARLO_SAMPLES; s++){
        /*===================== Initializing Filter ======================*/
    
        Eigen::VectorXd initial_state = Eigen::VectorXd::Ones(6) * 50;
        /*fix initial state MEAS_STD -> m, m/s | initial velocity is not eqal 1*/
        Eigen::VectorXd initial_target_state = target_traj[0].toCVStateVector();
        initial_state(0) = 500;
        initial_state(2) = 500;
        initial_state(4) = 500;

        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Zero(6,6);
        initial_cov.diagonal() << 1e6,400,1e6, 400,100000,400;

        /*initial EKF*/
        std::unique_ptr<GaussianFilter> ekf = std::make_unique<EKF>(initial_state, initial_cov, FRAME_STEP);

        /*initial UKF*/
        std::unique_ptr<GaussianFilter> ukf = std::make_unique<UKF>(initial_state, initial_cov, FRAME_STEP);

        /*initializing PCRLB*/
        PCRLB pcrlb(initial_cov, FRAME_STEP);

        /*initialize PF*/
        PF pf(FRAME_STEP);
        pf.setParticlesNumber(5000);
        pf.init_particles(initial_state, initial_cov);

        /*UPF*/
        UPF upf(FRAME_STEP);
        upf.setParticlesNumber(5000);
        upf.init_particles(initial_state, initial_cov);

        /*set State Transition F*/
        Eigen::MatrixXd F = init_Transit_Mat_CV(FRAME_STEP);
        ekf->setF(F);
        pf.setF(F);
        pcrlb.setF(F);
        upf.setF(F);

        /* ======================== Running Filters =======================*/
        for(int frame_count = 0; frame_count < MAX_FRAME; frame_count++){
            
            double frame_time = frame_count * FRAME_STEP;
            
            /*calculate measurements*/
            Eigen::Vector3d target_pos = target_traj[frame_count].getPos();
            Eigen::Vector3d obsv_pos = observer_traj[frame_count].getPos();
            Eigen::Vector3d dif_pos = target_pos - obsv_pos;

            double azimuth = std::atan2(dif_pos(1) , dif_pos(0));
            double elevation = std::atan2(dif_pos(2), dif_pos.head(2).norm());
    
            /*add noise in measurements*/
            azimuth += normal(0,MEAS_STD);azimuth = normalize_angle(azimuth);
            elevation += normal(0, MEAS_STD);
            Eigen::Vector2d measurement(azimuth, elevation);
    
            /* ------------------  EKF / UKF / PF / CRLB  ----------------------*/
            // std::cout << "[Frame : " << frame_count << " ] | ";
            ekf->predict(); /*EKF*/
            ekf->update(measurement, obsv_pos);
    
            ukf->predict(); /*ukf*/
            ukf->update(measurement, obsv_pos);

            pf.predict();
            pf.update_weights_focus(measurement, obsv_pos);
            pf.resample_if_needed();

            // upf.predict();
            // upf.update_weights_focus(measurement, obsv_pos);
            upf.importance_sampling(measurement, obsv_pos);
            upf.resample_if_needed();

            if(s == 0)     /*pcrlb*/
            { 
                pcrlb.update(target_traj[frame_count].toCVStateVector(), obsv_pos);
            }
            
            // Eigen::VectorXd cur_state = ekf->getMean();
            // std::cout << "measurement angles are ( " << azimuth << ", " << elevation << " ) | ";
            // std::cout << "GT : (" << target_pos(0) << ", " << target_pos(1) << ", " << target_pos(2) << ") "
            //             << " | prediction (" << cur_state(0) << ", " << cur_state(2) << ", " << cur_state(4) << ")\n ";  
            
            /* --------------------------  RMSE processing  ----------------------------*/
            if(s == 0){
                rmse_per_frame_list_ekf.push_back(rmse_per_frame(ekf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time));
                rmse_per_frame_list_ukf.push_back(rmse_per_frame(ukf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time));
                rmse_per_frame_list_pf.push_back(rmse_per_frame(pf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time));
                rmse_per_frame_list_upf.push_back(rmse_per_frame(upf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time));

                Eigen::Vector2d pcrlb_err = pcrlb.convertIToRmseVector();
                RMSE_per_Frame pcrlb_rmse(pcrlb_err(0), pcrlb_err(1), frame_time);
                rmse_per_frame_list_pcrlb.push_back(pcrlb_rmse);

            } else {
                rmse_per_frame_list_ekf[frame_count].pos += rmse_per_frame(ekf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time).pos;
                rmse_per_frame_list_ekf[frame_count].vel += rmse_per_frame(ekf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time).vel;
                
                rmse_per_frame_list_ukf[frame_count].pos += rmse_per_frame(ukf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time).pos;
                rmse_per_frame_list_ukf[frame_count].vel += rmse_per_frame(ukf->getMean(), target_traj[frame_count].toCVStateVector(), frame_time).vel;
                
                rmse_per_frame_list_pf[frame_count].pos += rmse_per_frame(pf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time).pos;
                rmse_per_frame_list_pf[frame_count].vel += rmse_per_frame(pf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time).vel;

                rmse_per_frame_list_upf[frame_count].pos += rmse_per_frame(upf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time).pos;
                rmse_per_frame_list_upf[frame_count].vel += rmse_per_frame(upf.get_estimated_state(), target_traj[frame_count].toCVStateVector(), frame_time).vel;
            }
        }
    }

    /*normalize*/
    for(int i = 0; i < MAX_FRAME; i++){
        rmse_per_frame_list_ekf[i].pos  /= (double)MONTE_CARLO_SAMPLES;
        rmse_per_frame_list_ekf[i].vel /=  (double)MONTE_CARLO_SAMPLES;

        rmse_per_frame_list_ukf[i].pos /= (double)MONTE_CARLO_SAMPLES;
        rmse_per_frame_list_ukf[i].vel /= (double)MONTE_CARLO_SAMPLES;

        rmse_per_frame_list_pf[i].pos /= (double)MONTE_CARLO_SAMPLES;
        rmse_per_frame_list_pf[i].vel /= (double)MONTE_CARLO_SAMPLES;

        rmse_per_frame_list_upf[i].pos /= (double)MONTE_CARLO_SAMPLES;
        rmse_per_frame_list_upf[i].vel /= (double)MONTE_CARLO_SAMPLES;
    }
    saveRmsePerFrameInJson(rmse_per_frame_list_ekf, "json/rmse_pos_ekf_015.json", "json/rmse_vel_ekf_015.json");
    saveRmsePerFrameInJson(rmse_per_frame_list_ukf, "json/rmse_pos_ukf_015.json", "json/rmse_vel_ukf_015.json");
    saveRmsePerFrameInJson(rmse_per_frame_list_pcrlb, "json/rmse_pos_pcrlb_015.json", "json/rmse_vel_pcrlb_015.json");
    saveRmsePerFrameInJson(rmse_per_frame_list_pf, "json/rmse_pos_pf_015.json", "json/rmse_vel_pf_015.json");
    saveRmsePerFrameInJson(rmse_per_frame_list_upf, "json/rmse_pos_upf_500_015.json", "json/rmse_vel_upf_500_015.json");
    
}