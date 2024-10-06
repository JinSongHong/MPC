#include "Kinematics.hpp"

Kinematics::Kinematics(int n)
{
    Leg_num = n;
    state = VectorXd::Zero(12);
}

Kinematics::~Kinematics()
{
}


void Kinematics::Cal_Kinematics()
{
    MatInertia_RW[0] =
        JzzR_thigh / (4 * pow(L, 2) * pow(sin(q2 / 2), 2));
    MatInertia_RW[1] = JzzR_couple / (2 * pow(L, 2) * sin(q2));
    MatInertia_RW[2] = JzzR_couple / (2 * pow(L, 2) * sin(q2));
    MatInertia_RW[3] =
    JzzR_shank / (4 * pow(L, 2) * pow(cos(q2 / 2), 2));
    
    //평행축 정리 적용(회전축에 대해 정의된 inertia)
    Jzz_thigh =
        Izz_thigh + m_thigh * pow(d_thigh, 2); // MoI of thigh w.r.t. HFE : thigh
    Jzz_shank =
        Izz_shank + m_shank * pow(d_shank, 2); // MoI of thigh w.r.t. KFE : shank

    // M1 = Izz_1 + m_1*(d_1)^2 + m_2*L^2
    M1 = Jzz_thigh + m_shank * pow(L, 2);
    // M2 = (m_2) * (d_2) * (L*cos(q(1)))
    M2 = m_shank * d_shank * L * cos(q2);
    // M12 = Izz_2 + m_2*(d_2)^2
    M12 = Jzz_shank;

    MatInertia_bi[0] = M1;
    MatInertia_bi[1] = M12;
    MatInertia_bi[2] = M12;
    MatInertia_bi[3] = M2;

    // RW 좌표계로 inertia 변환
    JzzR_thigh = Jzz_thigh + m_shank * pow(L, 2) + Jzz_shank - 2 * m_shank * d_shank * L * cos(q2);
    JzzR_couple = Jzz_thigh + m_shank * pow(L, 2) - Jzz_shank;
    JzzR_shank = Jzz_thigh + m_shank * pow(L, 2) + Jzz_shank + 2 * m_shank * d_shank * L * cos(q2);


    RWpos[0] = 2 * L * cos((q_bi[1] - q_bi[0]) / 2); // r
    RWpos[1] = (q_bi[0] + q_bi[1]) / 2;              // theta_r
    RWpos[2] = q_bi[0];                              // phi

    jacbRW <<       0,          L * cos(q2 / 2),    L * cos(q2 / 2),
              2 * L * cos(q2 / 2),      0,              0,
                    0,          L * sin(q2 / 2),    -L * sin(q2 / 2);


    jacbRW_trans = jacbRW.transpose(); 
    // cout << state_model->jacbRW_trans << endl; 
    // cout << state_model->jacbRW_trans(2,0) << endl;
    jacbRW_trans_inv = jacbRW_trans.inverse();

    RWvel = jacbRW * qdot_bi; 
    // cout << "Leg_num " << Leg_num << ": " << RWvel[0] << endl;
    // cout << "RWvel\n" << RWvel << endl;

}

void Kinematics::sensor_measure(const mjModel* m, mjData* d, int leg_no)
{

// joint position sensor data
    /*** (Serial) Joint position ***/
    q2 = d->sensordata[leg_no + 8];
    q[0] = d->sensordata[leg_no + 6];  // relative angle HAA angle
    q[1] = d->sensordata[leg_no + 7]; // (relative) HFE angle (thm)
    q[2] = d->sensordata[leg_no + 8]; // (relative) KFE angle (thb - thm)


    /*** Biarticular Transformation ***/
    q_bi[0] = d->sensordata[leg_no + 6];
    q_bi[1] = d->sensordata[leg_no + 7];                             // thm
    q_bi[2] = d->sensordata[leg_no + 7] + d->sensordata[leg_no + 8]; // thb

// angular velocity data_ real data
    qdot_bi[0] = d->qvel[leg_no + 6];
    qdot_bi[1] = d->qvel[leg_no + 7];
    qdot_bi[2] = d->qvel[leg_no + 7] + d->qvel[leg_no + 8];

//angular acceleration data_real data
    qddot_bi[0] = d->qacc[leg_no + 7];
    qddot_bi[1] = d->qacc[leg_no + 8];
    qddot_bi[2] = d->qacc[leg_no + 8] + d->qacc[leg_no + 9];



    
    // for (int i = 0; i < 3; i++)
    // {
    //     state_model->qdot_bi_tustin[i] =
    //         tustin_derivative(state_model->q_bi[i], state_model->q_bi_old[i], state_model->qdot_bi_tustin_old[i],
    //             param_tuning->freq_cut_D);
    //     state_model->qddot_bi_tustin[i] =
    //         tustin_derivative(state_model->qdot_bi_tustin[i], state_model->qdot_bi_tustin_old[i],
    //             state_model->qddot_bi_tustin_old[i], param_tuning->freq_cut_D);
    // }

    // trunk position
    for (int i = 0; i < 3; i++)
        pos_trunk_act[i+3] = d->sensordata[37 + i];

    // cout << pos_trunk_act[3] << endl;
    // Trunk orientation (Euler angle) ir global coordinates
    double qw = d->sensordata[40], qx = d->sensordata[41], qy = d->sensordata[42], qz = d->sensordata[43];
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    pos_trunk_act[0] = PI / 2 + atan2(sinr_cosp, cosr_cosp);
    // pos_trunk_act[3] = atan2(sinr_cosp, cosr_cosp);
    // pos_trunk_act[3] *= 180 / PI;

    double sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
    double cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
    pos_trunk_act[1] = 2 * atan2(sinp, cosp) - PI / 2;
    // pos_trunk_act[4] *= 180 / PI;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    pos_trunk_act[2] = atan2(siny_cosp, cosy_cosp);
    // pos_trunk_act[5] *= 180 / PI;

    // a = roll, b = pitch, r = yaw
    double a = pos_trunk_act[0], b = pos_trunk_act[1], r = pos_trunk_act[2];
 
    // cout << a << "   " << b << "   " << r << endl;
    // Trunk orientation (Rotation Matrix)
    R_trunk(0,0) = cos(b) * cos(r);
    R_trunk(0,1) = sin(a) * sin(b) * cos(r) - cos(a) * sin(r);
    R_trunk(0,2) = cos(a) * sin(b) * cos(r) + sin(a) * sin(r);
    R_trunk(1,0) = cos(b) * sin(r);
    R_trunk(1,1) = sin(a) * sin(b) * sin(r) + cos(a) * cos(r);
    R_trunk(1,2) = cos(a) * sin(b) * sin(r) - sin(a) * cos(r);
    R_trunk(2,0) = -sin(b);
    R_trunk(2,1) = sin(a) * cos(b);
    R_trunk(2,2) = cos(a) * cos(b);

    // trunk translational velocity
    for (int i = 0; i < 3; i++)
        vel_trunk_act[i + 3] = d->sensordata[34 + i];

    // trunk angular velocity
    for (int i = 0; i < 3; i++)
        vel_trunk_act[i] = d->sensordata[68 + i]; // global frame
    // vel_trunk_act[i] = d->sensordata[i];    // body frame

    // foot position represented in global frame
    for (int i = 0; i < 3; i++)
        {   
            // World frame
            // foot_pos_act[i] = d->sensordata[leg_no + 44 + i];
            // body frame
            foot_pos_B[i] = d->sensordata[leg_no + 44 + i] - pos_trunk_act[i+3]; 
        }

    // Vector3d foot_pos;

    // foot_pos << d->site_xpos[6 + 6*Leg_num], d->site_xpos[6+6*Leg_num+1], d->site_xpos[6+6*Leg_num+2];  
    
    // cout << "site: " << Leg_num << endl << foot_pos << endl;

        foot_pos_skew << 0,         -foot_pos_B[2],  foot_pos_B[1],
                     foot_pos_B[2],       0,        -foot_pos_B[0],
                    -foot_pos_B[1],  foot_pos_B[0],       0;

        // foot_pos_skew << 0,         -foot_pos[2],  foot_pos[1],
        //             foot_pos[2],       0,        -foot_pos[0],
        //             -foot_pos[1],  foot_pos[0],       0;

    

    // cout << "foot_pos\n" << foot_pos[0] << endl;
    // if(Leg_num == 0)
    // cout << "foot_pos_B\n" << foot_pos_B << endl;
    

    for(int i = 0; i < 6; i++)
    {state[i] = pos_trunk_act[i];}
    for(int i = 0; i < 6; i++)
    {state[i+6] = vel_trunk_act[i];}

    
    I_B  << 0.1619, 0, 0,
            0, 1.1622, 0,
            0, 0, 1.0325;

    I_W = R_trunk * I_B * R_trunk.transpose(); 
    

}




Vector3d Kinematics::get_vel_error(Vector3d RWvel_ref)
{
    RWvel_error = RWvel_ref - RWvel;
    
    return RWvel_error;
}

void Kinematics::update_old_state()
{
    RWvel_old = RWvel;
    RWvel_error_old = RWvel_error;
}

