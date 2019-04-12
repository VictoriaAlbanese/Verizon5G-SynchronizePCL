////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: MLS.h
//
// Purpose: Header file for cuda MLS stuff, from this source
// https://github.com/colincsl/Kinect-Projects/blob/master/MovingLeastSquares_PCL/MLS.h
//
////////////////////////////////////////////////////////////////

#ifndef ITERATE_H
#define ITERATE_H

#include <boost/shared_ptr.hpp>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <pcl/cuda/common/eigen.h>
#include <pcl/cuda/cutil.h>
#include <pcl/cuda/cutil_math.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/host_device.h>
#include <pcl/cuda/point_cloud.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/device_reference.h>
#include <thrust/fill.h>
#include <thrust/functional.h>
#include <thrust/gather.h>
#include <thrust/host_vector.h>
#include <thrust/replace.h>
#include <thrust/sequence.h>
#include <thrust/transform.h>
#include <thrust/transform_reduce.h>
#include <vector>

#define MEGA 1048576
#define NN_CONNECTIVITY 4
#define SMOOTHNESS 0.01

using namespace std;
using namespace pcl::cuda;
using namespace thrust;
using pcl::cuda::PointCloudAOS;
using pcl::cuda::Device;
using pcl::cuda::PointXYZRGB;

void thrustPCL();
void thrustPCL_AOS( PointCloudAOS<Device>::Ptr x, 
                    PointCloudAOS<Device>::Ptr y, 
                    int nn_connect, 
                    float smoothness);

struct tTestKernel
{
    const float a;
    tTestKernel(float _a):a(_a){};
    __host__ __device__
    float operator()(const float& x, const float& y) const;
};

struct PCLKernel
{
    PCLKernel(){};
    template <typename Tuple>
    __host__ __device__
    pcl::cuda::PointXYZRGB operator()(const Tuple &x) const;	
};

struct MovingLeastSquares
{
    typedef boost::shared_ptr<PointCloudAOS<Device> > CloudVar;

    pcl::cuda::PointXYZRGB *points;
    int nn_connectivity;
    int nn_count_;
    float smoothness;

    MovingLeastSquares(const CloudVar &data_, int nn_connect_, float smoothness_)
        : points(raw_pointer_cast(data_->points.data().get()))
        , nn_connectivity(nn_connect_)
        , smoothness(smoothness_)
        , nn_count_(nn_connect_*nn_connect_) 
    {
        //points = raw_pointer_cast<PointXYZRGB>(&data_->points[0]);
        //points = raw_pointer_cast(data_->points.data().get());

    }

    template <typename Tuple>
    __host__ __device__
    pcl::cuda::PointXYZRGB operator()(const Tuple &x) const;
};

#endif // ITERATE_H

////////////////////////////////////////////////////////////////
