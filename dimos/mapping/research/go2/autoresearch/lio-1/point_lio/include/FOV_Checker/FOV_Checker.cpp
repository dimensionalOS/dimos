#include "FOV_Checker.h"

FOV_Checker::FOV_Checker(){
    // fp = fopen("/home/ecstasy/catkin_ws/fov_data.csv","w");
    // fprintf(fp,"cur_pose_x,cur_pose_y,cur_pose_z,axis_x,axis_y,axis_z,theta,depth\n");
    // fclose(fp);
}

FOV_Checker::~FOV_Checker(){

}

void FOV_Checker::Set_Env(BoxPointType env_param){
    env = env_param;
}

void FOV_Checker::Set_BoxLength(double box_len_param){
    box_length = box_len_param;
}

void round_v3d(Eigen::Vector3d &vec, int decimal){
    double tmp;
    int t;
    for (int i = 0; i < 3; i++){
        t = pow(10,decimal);
        tmp = round(vec(i)*t);
        vec(i) = tmp/t;
    }
    return;
}

void FOV_Checker::check_fov(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, vector<BoxPointType> &boxes){
    round_v3d(cur_pose,4);
    round_v3d(axis,3);
    axis = axis/axis.norm();
    // fp = fopen("/home/ecstasy/catkin_ws/fov_data.csv","a");
    // fprintf(fp,"%f,%f,%f,%f,%f,%f,%0.4f,%0.1f,",cur_pose(0),cur_pose(1),cur_pose(2),axis(0),axis(1),axis(2),theta,depth);
    // fclose(fp);
    // cout << "cur_pose: " << cur_pose.transpose() << endl;
    // cout<< "axis: " << axis.transpose() << endl;
    // cout<< "theta: " << theta << " depth: " << depth << endl;
    // cout<< "env: " << env.vertex_min[0] << " " << env.vertex_max[0] << endl;
    double axis_angle[6], min_angle, gap, plane_u_min, plane_u_max;
    Eigen::Vector3d plane_w, plane_u, plane_v, center_point, start_point, box_p;
    Eigen::Vector3d box_p_min, box_p_max;
    int i, j, k, index, maxn, start_i, max_uN, max_vN, max_ulogN, u_min, u_max;
    bool flag = false, box_found = false;
    boxes.clear();
    BoxPointType box;
    axis_angle[0] = acos(axis(0));
    axis_angle[1] = acos(axis(1));
    axis_angle[2] = acos(axis(2));
    axis_angle[3] = acos(-axis(0));
    axis_angle[4] = acos(-axis(1));
    axis_angle[5] = acos(-axis(2));
    index = 1;
    min_angle = axis_angle[0];
    for (i=1;i<6;i++){
        if (axis_angle[i]<min_angle){
            min_angle = axis_angle[i];
            index = i+1;
        }
    }
    switch (index){
        case 1:
            // YZ plane
            plane_w = Eigen::Vector3d(1,0,0);
            plane_u = Eigen::Vector3d(0,1,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = floor(cur_pose(0)/box_length + 0.5 + eps_value) * box_length + 0.5 * box_length - cur_pose(0);
            maxn = ceil((env.vertex_max[0]-cur_pose(0))/box_length) +1;
            start_i = 0;
            break;
        case 2:
            // XZ plane
            plane_w = Eigen::Vector3d(0,1,0);
            plane_u = Eigen::Vector3d(1,0,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = floor(cur_pose(1)/box_length + 0.5 + eps_value) * box_length + 0.5 * box_length - cur_pose(1);
            maxn = ceil((env.vertex_max[1]-cur_pose(1))/box_length) +1;
            start_i = 0;
            break;
        case 3:
            // XY plane
            plane_w = Eigen::Vector3d(0,0,1);
            plane_u = Eigen::Vector3d(1,0,0);
            plane_v = Eigen::Vector3d(0,1,0);
            gap = floor(cur_pose(2)/box_length + 0.5 + eps_value) * box_length + 0.5 * box_length - cur_pose(2);
            maxn = ceil((env.vertex_max[2]-cur_pose(2))/box_length) +1;
            start_i = 0;
            break;
        case 4:
            // YZ plane
            plane_w = Eigen::Vector3d(1,0,0);
            plane_u = Eigen::Vector3d(0,1,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = ceil(cur_pose(0)/box_length - 0.5 - eps_value) * box_length - 0.5 * box_length - cur_pose(0);
            maxn = ceil((cur_pose(0)-env.vertex_min[0])/box_length) +1;
            start_i = 1;
            break;
        case 5:
            // XZ plane
            plane_w = Eigen::Vector3d(0,1,0);
            plane_u = Eigen::Vector3d(1,0,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = ceil(cur_pose(1)/box_length - 0.5 - eps_value) * box_length - 0.5 * box_length - cur_pose(1);
            maxn = ceil((cur_pose(1)-env.vertex_min[1])/box_length) +1;
            start_i = 1;
            break;
        case 6:
            // XY plane
            plane_w = Eigen::Vector3d(1,0,0);
            plane_u = Eigen::Vector3d(0,1,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = ceil(cur_pose(2)/box_length - 0.5 - eps_value) * box_length - 0.5 * box_length - cur_pose(2);
            maxn = ceil((cur_pose(2)-env.vertex_min[2])/box_length) +1;
            start_i = 1;
            break;
        default:
            // YZ plane
            plane_w = Eigen::Vector3d(1,0,0);
            plane_u = Eigen::Vector3d(0,1,0);
            plane_v = Eigen::Vector3d(0,0,1);
            gap = ceil(cur_pose(0)/box_length + 0.5 + eps_value) * box_length + 0.5 * box_length - cur_pose(0);
            maxn = ceil((env.vertex_max[0]-cur_pose(0))/box_length) +1;
            start_i = 0;
            break;
    }
    for (i=start_i; i<=maxn; i++){
        center_point = cur_pose + (abs(gap) + (i-1) * box_length)/cos(min_angle) * axis;
        if (index == 1 || index == 4){
            start_point = Eigen::Vector3d(center_point(0),floor(center_point(1)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length,floor(center_point(2)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length);
            max_uN = ceil((env.vertex_max[1]-env.vertex_min[1])/box_length);
            max_ulogN = floor(log2(max_uN));
            max_vN = ceil((env.vertex_max[2]-env.vertex_min[2])/box_length);
            plane_u_min = env.vertex_min[1];
            plane_u_max = env.vertex_max[1];
        } else {
            if (index == 2 || index == 5){
                start_point = Eigen::Vector3d(floor(center_point(0)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length, center_point(1), floor(center_point(2)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length);
                max_uN = ceil((env.vertex_max[0]-env.vertex_min[0])/box_length);
                max_ulogN = floor(log2(max_uN));
                max_vN = ceil((env.vertex_max[2]-env.vertex_min[2])/box_length);
                plane_u_min = env.vertex_min[0];
                plane_u_max = env.vertex_max[0];
            } else {
                start_point = Eigen::Vector3d(floor(center_point(0)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length, floor(center_point(1)/box_length + 0.5 + eps_value)*box_length - 0.5 * box_length, center_point(2));
                max_uN = ceil((env.vertex_max[1]-env.vertex_min[1])/box_length);
                max_ulogN = floor(log2(max_uN));
                max_vN = ceil((env.vertex_max[2]-env.vertex_min[2])/box_length);
                plane_u_min = env.vertex_min[1];
                plane_u_max = env.vertex_max[1];
            }
        }
        flag = false;
        for (j = 1; j <= max_vN; j++){
            k = max_ulogN;
            u_min = 0;
            box_p_min = start_point.cwiseProduct(plane_w + plane_v) + plane_u * plane_u_min + plane_v * box_length * (j-1);
            box_p_max = plane_u * plane_u_max + start_point.cwiseProduct(plane_w + plane_v) + plane_v * box_length * j +  plane_w * box_length;
            //printf("---- UPSIDE (%0.3f,%0.3f,%0.3f),(%0.3f,%0.3f,%0.3f)\n",box_p_min[0],box_p_min[1],box_p_min[2],box_p_max[0],box_p_max[1],box_p_max[2]);

            while (k>=0){
                box_p = box_p_min + plane_u * box_length * (u_min + pow(2,k)) + plane_v * box_length + plane_w * box_length;
                box.vertex_min[0] = box_p_min(0);
                box.vertex_min[1] = box_p_min(1);
                box.vertex_min[2] = box_p_min(2);
                box.vertex_max[0] = box_p(0);
                box.vertex_max[1] = box_p(1);
                box.vertex_max[2] = box_p(2);
                if (!check_box(cur_pose, axis, theta, depth, box)) u_min = u_min + pow(2,k);
                k = k-1;
            }
            k = max_ulogN;
            u_max = 0;
            while (k>=0){
                box_p = box_p_max - plane_u * box_length * (u_max + pow(2,k)) - plane_v * box_length - plane_w * box_length;
                box.vertex_min[0] = box_p(0);
                box.vertex_min[1] = box_p(1);
                box.vertex_min[2] = box_p(2);
                box.vertex_max[0] = box_p_max(0);
                box.vertex_max[1] = box_p_max(1);
                box.vertex_max[2] = box_p_max(2);
                if (!check_box(cur_pose, axis, theta, depth, box)) u_max = u_max + pow(2,k);

                k = k-1;
            }
            u_max = max(0, max_uN - u_max - 1);
            box_found = false;
            //printf("---- u_min -> u_max: %d->%d\n",u_min,u_max);
            for (k = u_min; k <= u_max; k++){
                box_p = box_p_min  + plane_u * box_length * k;
                box.vertex_min[0] = box_p(0);
                box.vertex_min[1] = box_p(1);
                box.vertex_min[2] = box_p(2);
                box.vertex_max[0] = box_p(0) + box_length;
                box.vertex_max[1] = box_p(1) + box_length;
                box.vertex_max[2] = box_p(2) + box_length;
                if (check_box_in_env(box)){
                    //printf("---- FOUND: (%0.3f,%0.3f,%0.3f),(%0.3f,%0.3f,%0.3f)\n",box.vertex_min[0],box.vertex_min[1],box.vertex_min[2],box.vertex_max[0],box.vertex_max[1],box.vertex_max[2]);
                    box_found = true;
                    boxes.push_back(box);
                }
            }
            if (box_found) {
                flag = true;
            } else {
                if (j>1) break;
            }
        }
        for (j = 1; j <= max_vN; j++){
            k = max_ulogN;
            u_min = 0;
            box_p_min = start_point.cwiseProduct(plane_w + plane_v) + plane_u * plane_u_min - plane_v * box_length * j;
            box_p_max = plane_u * plane_u_max + start_point.cwiseProduct(plane_w + plane_v) - plane_v * box_length * (j-1) +  plane_w * box_length;
            //printf("---- DOWNSIDE (%0.3f,%0.3f,%0.3f),(%0.3f,%0.3f,%0.3f)\n",box_p_min[0],box_p_min[1],box_p_min[2],box_p_max[0],box_p_max[1],box_p_max[2]);

            while (k>=0){
                box_p = box_p_min + plane_u * box_length * (u_min + pow(2,k)) + plane_v * box_length + plane_w * box_length;
                box.vertex_min[0] = box_p_min(0);
                box.vertex_min[1] = box_p_min(1);
                box.vertex_min[2] = box_p_min(2);
                box.vertex_max[0] = box_p(0);
                box.vertex_max[1] = box_p(1);
                box.vertex_max[2] = box_p(2);
                if (!check_box(cur_pose, axis, theta, depth, box)) u_min = u_min + pow(2,k);
                k = k-1;
            }
            k = max_ulogN;
            u_max = 0;
            while (k>=0){
                box_p = box_p_max - plane_u * box_length * (u_max + pow(2,k)) - plane_v * box_length - plane_w * box_length;
                box.vertex_min[0] = box_p(0);
                box.vertex_min[1] = box_p(1);
                box.vertex_min[2] = box_p(2);
                box.vertex_max[0] = box_p_max(0);
                box.vertex_max[1] = box_p_max(1);
                box.vertex_max[2] = box_p_max(2);
                if (!check_box(cur_pose, axis, theta, depth, box)) {
                    u_max = u_max + pow(2,k);
                    // printf("-------- Not Included: (%0.3f,%0.3f,%0.3f),(%0.3f,%0.3f,%0.3f)\n",box.vertex_min[0],box.vertex_min[1],box.vertex_min[2],box.vertex_max[0],box.vertex_max[1],box.vertex_max[2]);
                }

                k = k-1;
            }
            u_max = max(0, max_uN - u_max - 1);
            //printf("---- u_min -> u_max: %d->%d\n",u_min,u_max);
            box_found = 0;
            for (k = u_min; k <= u_max; k++){
                box_p = box_p_min  + plane_u * box_length * k;
                box.vertex_min[0] = box_p(0);
                box.vertex_min[1] = box_p(1);
                box.vertex_min[2] = box_p(2);
                box.vertex_max[0] = box_p(0) + box_length;
                box.vertex_max[1] = box_p(1) + box_length;
                box.vertex_max[2] = box_p(2) + box_length;
                if (check_box_in_env(box)){
                    //printf("---- FOUND: (%0.3f,%0.3f,%0.3f),(%0.3f,%0.3f,%0.3f)\n",box.vertex_min[0],box.vertex_min[1],box.vertex_min[2],box.vertex_max[0],box.vertex_max[1],box.vertex_max[2]);
                    box_found = 1;
                    boxes.push_back(box);
                }
            }
            if (box_found) {
                flag = true;
            } else {
                if (j>1) break;
            }
        }
        if (!flag && i>0) break;
    }
}

bool FOV_Checker::check_box(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box){
    Eigen::Vector3d vertex[8];
    bool s;
    vertex[0] = Eigen::Vector3d(box.vertex_min[0], box.vertex_min[1], box.vertex_min[2]);
    vertex[1] = Eigen::Vector3d(box.vertex_min[0], box.vertex_min[1], box.vertex_max[2]);
    vertex[2] = Eigen::Vector3d(box.vertex_min[0], box.vertex_max[1], box.vertex_min[2]);
    vertex[3] = Eigen::Vector3d(box.vertex_min[0], box.vertex_max[1], box.vertex_max[2]);
    vertex[4] = Eigen::Vector3d(box.vertex_max[0], box.vertex_min[1], box.vertex_min[2]);
    vertex[5] = Eigen::Vector3d(box.vertex_max[0], box.vertex_min[1], box.vertex_max[2]);
    vertex[6] = Eigen::Vector3d(box.vertex_max[0], box.vertex_max[1], box.vertex_min[2]);
    vertex[7] = Eigen::Vector3d(box.vertex_max[0], box.vertex_max[1], box.vertex_max[2]);
    for (int i = 0; i < 8; i++){
        if (check_point(cur_pose, axis, theta, depth, vertex[i])){
            return true;
        }
    }
    Eigen::Vector3d center_point = (vertex[7]+vertex[0])/2.0;
    if (check_point(cur_pose, axis, theta, depth, center_point)){
        return true;
    }
    PlaneType plane[6];
    plane[0].p[0] = vertex[0];
    plane[0].p[1] = vertex[2];
    plane[0].p[2] = vertex[1];
    plane[0].p[3] = vertex[3];

    plane[1].p[0] = vertex[0];
    plane[1].p[1] = vertex[4];
    plane[1].p[2] = vertex[2];
    plane[1].p[3] = vertex[6];

    plane[2].p[0] = vertex[0];
    plane[2].p[1] = vertex[4];
    plane[2].p[2] = vertex[1];
    plane[2].p[3] = vertex[5];

    plane[3].p[0] = vertex[4];
    plane[3].p[1] = vertex[6];
    plane[3].p[2] = vertex[5];
    plane[3].p[3] = vertex[7];

    plane[4].p[0] = vertex[2];
    plane[4].p[1] = vertex[6];
    plane[4].p[2] = vertex[3];
    plane[4].p[3] = vertex[7];

    plane[5].p[0] = vertex[1];
    plane[5].p[1] = vertex[5];
    plane[5].p[2] = vertex[3];
    plane[5].p[3] = vertex[7];
    if (check_surface(cur_pose, axis, theta, depth, plane[0]) || check_surface(cur_pose, axis, theta, depth, plane[1]) || check_surface(cur_pose, axis, theta, depth, plane[2]) || check_surface(cur_pose, axis, theta, depth, plane[3]) || check_surface(cur_pose, axis, theta, depth, plane[4]) || check_surface(cur_pose, axis, theta, depth, plane[5]))
        s = 1;
    else
        s = 0;
    return s;
}

bool FOV_Checker::check_surface(Eigen::Vector3d cur_pose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane){
    Eigen::Vector3d plane_p, plane_u, plane_v, plane_w, pc, p, vec;
    bool s;
    double t, vec_dot_u, vec_dot_v;
    plane_p = plane.p[0];
    plane_u = plane.p[1] - plane_p;
    plane_v = plane.p[2] - plane_p;
    if (check_line(cur_pose, axis, theta, depth, plane_p, plane_u) || check_line(cur_pose, axis, theta, depth, plane_p, plane_v) || check_line(cur_pose, axis, theta, depth, plane_p + plane_u, plane_v) || check_line(cur_pose, axis, theta, depth, plane_p + plane_v, plane_u)){
        s = 1;
        return s;
    }
    pc = plane_p + (plane.p[3]-plane.p[0])/2;
    if (check_point(cur_pose, axis, theta, depth, pc)){
        s = 1;
        return s;
    }
    plane_w = plane_u.cross(plane_v);
    p = plane_p - cur_pose;
    t = (p.dot(plane_w))/(axis.dot(plane_w));
    vec = cur_pose + t * axis - plane_p;
    vec_dot_u = vec.dot(plane_u)/plane_u.norm();
    vec_dot_v = vec.dot(plane_v)/plane_v.norm();
    if (t>=-eps_value && t<=depth && vec_dot_u>=-eps_value && vec_dot_u<=plane_u.norm() && vec_dot_v>=-eps_value && vec_dot_v <= plane_v.norm())
        s = 1;
    else
        s = 0;
    return s;
}

bool FOV_Checker::check_line(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d line_p, Eigen::Vector3d line_vec){
    Eigen::Vector3d p, vec_1, vec_2;
    double xl, yl, zl, xn, yn, zn, dot_1, dot_2, ln, pn, pl, l2, p2;
    double A, B, C, delta, t1, t2;
    bool s;
    p = line_p - cur_pose;
    xl = line_vec(0); yl = line_vec(1); zl = line_vec(2);
    xn = axis(0); yn = axis(1); zn = axis(2);
    vec_1 = line_p - cur_pose;
    vec_2 = line_p + line_vec - cur_pose;
    dot_1 = vec_1.dot(axis);
    dot_2 = vec_2.dot(axis);
    //printf("xl yl zl: %0.4f, %0.4f, %0.4f\n", xl, yl, zl);
    //printf("xn yn zn: %0.4f, %0.4f, %0.4f\n", xn, yn, zn);
    //printf("dot_1, dot_2, %0.4f, %0.4f\n",dot_1, dot_2);
    if ((dot_1<0 && dot_2<0) || (dot_1>depth && dot_2>depth)){
        s = false;
        return s;
    }
    ln = xl*xn+yl*yn+zl*zn;
    pn = p(0)*xn+p(1)*yn+p(2)*zn;
    pl = p(0)*xl+p(1)*yl+p(2)*zl;
    l2 = xl*xl+yl*yl+zl*zl;
    p2 = p.norm()*p.norm();
    //printf("ln: %0.4f\n",ln);
    //printf("pn:%0.4f\n",pn);
    //printf("pl:%0.4f\n",pl);
    //printf("l2:%0.4f\n",l2);
    //printf("p2:%0.4f\n",p2);
    //printf("theta, cos(theta):%0.4f %0.4f\n",theta,cos(theta));
    A = ln * ln - l2 * cos(theta) * cos(theta);
    B = 2 * pn * ln - 2 * cos(theta) * cos(theta)*pl;
    C = pn * pn - p2 * cos(theta) * cos(theta);
    //printf("A:%0.4f, B:%0.4f, C:%0.4f\n", A,B,C);
    if (!(fabs(A)<=eps_value)){
        delta = B*B - 4*A*C;
        //printf("delta: %0.4f\n",delta);
        if (delta <= eps_value){
            if (A < -eps_value){
                s = false;
                return s;
            }
            else{
                s = true;
                return s;
            }
        } else {
            double sqrt_delta = sqrt(delta);
            t1 = (-B - sqrt_delta)/(2*A);
            t2 = (-B + sqrt_delta)/(2*A);
            if (t1>t2) swap(t1,t2);
            //printf("t1,t2: %0.4f,%0.4f\n",t1,t2);
            // printf("%d\n",check_point(cur_pose, axis, theta, depth, line_p + line_vec * t1));
            if ((t1>=-eps_value && t1<=1+eps_value) && check_point(cur_pose, axis, theta, depth, line_p + line_vec * t1)){
                s = true;
                return s;
            }
            // printf("%d\n",check_point(cur_pose, axis, theta, depth, line_p + line_vec * t2));
            if ((t2>=-eps_value && t2<=1+eps_value) && check_point(cur_pose, axis, theta, depth, line_p + line_vec * t2)){
                s = true;
                return s;
            }
            if (A>-eps_value && (t2<eps_value || t1>1-eps_value)){
                s = true;
                return s;
            }
            if (A<eps_value && t1<eps_value && t2>1-eps_value){
                s = true;
                return s;
            }
            s = false;
        }
    } else{
        if (!(fabs(B)<=eps_value)){
            s = (B>-eps_value && -C/B<=1+eps_value) || (B<eps_value && -C/B>=-eps_value);
            return s;
        }
        else {
            s = C>=-eps_value;
            return s;
        }
    }
    return false;
}

bool FOV_Checker::check_point(Eigen::Vector3d cur_pose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point){
    Eigen::Vector3d vec;
    double proj_len;
    bool s;
    vec = point-cur_pose;
    if (vec.transpose()*vec < 0.4 * box_length * box_length){
        return true;
    }
    proj_len = vec.dot(axis);
    if (proj_len > depth){
        s = false;
        return s;
    }
    //printf("acos: %0.4f\n",acos(proj_len/vec.norm()));
    if (fabs(vec.norm()) <= 1e-4 || acos(proj_len/vec.norm()) <= theta + 0.0175)
        s = true;
    else
        s = false;
    return s;
}

bool FOV_Checker::check_box_in_env(BoxPointType box){
    if (box.vertex_min[0] >= env.vertex_min[0]-eps_value && box.vertex_min[1] >= env.vertex_min[1]-eps_value && box.vertex_min[2] >= env.vertex_min[2]-eps_value && box.vertex_max[0]<= env.vertex_max[0]+eps_value && box.vertex_max[1]<= env.vertex_max[1]+eps_value && box.vertex_max[2]<= env.vertex_max[2]+eps_value){
        return true;
    } else {
        return false;
    }
}
