////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: MLS.cu
//
// Purpose: Cuda MLS stuff, from this source
// https://github.com/colincsl/Kinect-Projects/blob/master/MovingLeastSquares_PCL/MLS.h
//
////////////////////////////////////////////////////////////////

#include "MLS.h"

void thrustPCL_AOS(boost::shared_ptr<PointCloudAOS<Device> > cloud,
              PointCloudAOS<Device>::Ptr out,
              int nn_connectivity,
              float smoothness)
{
    const int size = 640*480;
    device_vector<int> indices(size);
    thrust::sequence(indices.begin(), indices.end());

    MovingLeastSquares kernel = MovingLeastSquares(cloud, nn_connectivity, smoothness);

    thrust::transform(make_zip_iterator(make_tuple(cloud->points.begin(), indices.begin())),
                      make_zip_iterator(make_tuple(cloud->points.end(), indices.end())), 
                      out->points.begin(),
                      kernel);
}

template <typename Tuple>
__host__ __device__
PointXYZRGB MovingLeastSquares::operator()(const Tuple &data) const
{
    PointXYZRGB point = thrust::get<0>(data);
    int index = thrust::get<1>(data);

    PointXYZRGB point2, pOut, nn_;

    const int nn_count = 49;
    int stride[] =  {-1923, -1922, -1921, -1920, -1919, -1918, -1917,\
                    -1283, -1282, -1281, -1280, -1279, -1278, 1977,\
                    -641, -642, -641, -640, -639,-638,-637, \
                    -3, -2, -1, 0, 1, 2, 3,\
                     638, 639, 640, 641, 642, 643,\
                     1277, 1278, 1279, 1280, 1281, 1282, 1283, \
                     1917, 1918, 1919, 1920, 1921, 1922, 1923};

    float3 centroid = make_float3(0.0,0.0,0.0);
    float3 neighbors[nn_count];

    // Find centroid --------------------------------------------

    int current_ind=0;
    int ind_count=0;
    int max_ind = 640*480;
    //float thresh = .01*nn_connectivity;
    
    for (int i=0; i<nn_count; i++)
    {
        current_ind = index+stride[i];
        if (current_ind >= 0 && current_ind < max_ind)
        {
            nn_ = points[current_ind];

            if (nn_.x > 0.0);
            {
                centroid.x += nn_.x;
                centroid.y += nn_.y;
                centroid.z += nn_.z;

                neighbors[ind_count] = make_float3(nn_.x, nn_.y, nn_.z);
                ind_count++;
            }
        }
    }

    if (ind_count < 3) return pOut;

    centroid.x /= (float)ind_count;
    centroid.y /= (float)ind_count;
    centroid.z /= (float)ind_count;            
        
    // Calculate covariance -----------------------------------

    CovarianceMatrix cov;
    cov.data[0] = make_float3(0,0,0);
    cov.data[1] = make_float3(0,0,0);
    cov.data[2] = make_float3(0,0,0);

    float3 tmp_nn;
    for (int k=0; k<ind_count; k++)
    {
        tmp_nn = neighbors[k];
        cov.data[0].x += (tmp_nn.x-centroid.x)*(tmp_nn.x-centroid.x);
        cov.data[0].y += (tmp_nn.x-centroid.x)*(tmp_nn.y-centroid.y);
        cov.data[0].z += (tmp_nn.x-centroid.x)*(tmp_nn.z-centroid.z);
        cov.data[1].y += (tmp_nn.y-centroid.y)*(tmp_nn.y-centroid.y);
        cov.data[1].z += (tmp_nn.y-centroid.y)*(tmp_nn.z-centroid.z);
        cov.data[2].z += (tmp_nn.z-centroid.z)*(tmp_nn.z-centroid.z);
    }
        
    cov.data[0].x /= (ind_count-1);
    cov.data[0].y /= (ind_count-1);
    cov.data[0].z /= (ind_count-1);
    cov.data[1].y /= (ind_count-1);
    cov.data[1].z /= (ind_count-1);
    cov.data[2].z /= (ind_count-1);

    // fill in the lower triangle (symmetry)
    cov.data[1].x = cov.data[0].y;
    cov.data[2].x = cov.data[0].z;
    cov.data[2].y = cov.data[1].z;

    // Eigen Stuff? -----------------------------------

    CovarianceMatrix evecs;
    float3 evals;

    pcl::cuda::eigen33 (cov, evecs, evals);

    float3 pointC;
    float3 normal = evecs.data[0];
    float eigenvalue = evals.z;

    pointC.x = point.x; 
    pointC.y = point.y; 
    pointC.z = point.z;
        
    float model_coeff = -1*(normal.x*centroid.x + normal.y*centroid.y + normal.z*centroid.z);
    float distance = pointC.x*normal.x + pointC.y*normal.y + pointC.z*normal.z + model_coeff;
    pointC -= distance * normal;

    float curvature = cov.data[0].x+cov.data[1].y+cov.data[2].z; // curv = tr(covariance)
    if (curvature != 0) curvature = fabs(eigenvalue / curvature);

    float nn_dist[nn_count];
    for (int i=0; i<ind_count; i++)
    {
        neighbors[i] -= pointC;
        nn_dist[i] = neighbors[i].x*neighbors[i].x + neighbors[i].y*neighbors[i].y + neighbors[i].z*neighbors[i].z;
    }

    // Init polynomial params - assume 3 coeffs for now
    const int nr_coeff = 3; // number of coeffs in polynomial
    const float sqr_gauss_param = smoothness;

    float weight_vec[nn_count];
    float f_vec[nn_count];
    float3 c_vec;
    float3 P[nn_count];
    float3 P_weight[nn_count];

    //Local coordinate system
    float3 v = unitOrthogonal(normal);
    float3 u = cross(normal, v);

    float u_coord, v_coord, u_pow, v_pow;
    for (int i=0; i<ind_count; i++)
    {
        // Compute weight
        weight_vec[i] = exp(-nn_dist[i] / sqr_gauss_param);

        // Transform coords
        u_coord = neighbors[i].x*u.x + neighbors[i].y*u.y +  neighbors[i].z*u.z;
        v_coord = neighbors[i].x*v.x + neighbors[i].y*v.y +  neighbors[i].z*v.z;
        f_vec[i]= dot(neighbors[i], normal);

        u_pow = 1;
        for(int i2=0; i2<nr_coeff-1; i2++)
        {
            v_pow=1;
            P[i].x = u_pow*v_pow;
            v_pow *= v_coord;
            P[i].y = u_pow*v_pow;
            v_pow *= v_coord;
            P[i].z = u_pow*v_pow;
            v_pow *= v_coord;

            u_pow *= u_coord;
        }
    }

    // P is NNx3
    // P_weight is NNx3
    // P_weight_Pt is 3x3
    for (int i=0; i<ind_count; i++)
    {
        P_weight[i].x = P[i].x*weight_vec[i];
        P_weight[i].x = P[i].x*weight_vec[i];
        P_weight[i].x = P[i].x*weight_vec[i];
    }

    c_vec.x=0.0; c_vec.y=0.0; c_vec.z=0.0;
    for (int i=0; i<ind_count; i++)
    {
        c_vec.x += P_weight[i].x*f_vec[i];
        c_vec.y += P_weight[i].y*f_vec[i];
        c_vec.z += P_weight[i].z*f_vec[i];
    }

    pointC += c_vec.x*normal;

    pOut.x = pointC.x;
    pOut.y = pointC.y;
    pOut.z = pointC.z;
    pOut.rgb.b = 120;
    pOut.rgb.r = 120;

    return pOut;
};

////////////////////////////////////////////////////////////////
