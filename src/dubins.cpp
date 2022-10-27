// Copyright (c) 2008-2014, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DUBINS_CPP
#define DUBINS_CPP

#include "dubins.h"
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <assert.h>

#define EPSILON (10e-10)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

// The three segment types a path can be made up of
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

namespace HybridAStar{

// The segment types for each of the Path types
const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

// 六种可能的dubins曲线
DubinsWord dubins_words[] = {
    dubins_LSL,
    dubins_LSR,
    dubins_RSL,
    dubins_RSR,
    dubins_RLR,
    dubins_LRL,
};

// 
#define UNPACK_INPUTS(alpha, beta)     \
    double sa = sin(alpha);            \
    double sb = sin(beta);             \
    double ca = cos(alpha);            \
    double cb = cos(beta);             \
    double c_ab = cos(alpha - beta);   \

// 
#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;


/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */

// 
double fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

// 
double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

// 归一化参数的dubins曲线生成
int dubins_init_normalised( double alpha,   // 转化坐标后 起点与坐标的夹角
                            double beta,    // 转化坐标后 终点与坐标的夹角
                            double d,       // ？？ 起点到终点距离的平方 / 最小转弯半径
                            DubinsPath* path)
{
    // 初始化最有代价为无穷大
    double best_cost = INFINITY;
    // 记录最优的那一种曲线是哪条
    int    best_word;
    // 
    int    i;
    // 初始化最优的那条
    best_word = -1;
    
    // 遍历6种 可能 找出最优的那种
    for( i = 0; i < 6; i++ ) {

        double params[3];

        //     DubinsWord dubins_words[] = {
        //     dubins_LSL,
        //     dubins_LSR,
        //     dubins_RSL,
        //     dubins_RSR,
        //     dubins_RLR,
        //     dubins_LRL,
        // };

        // 利用归一化后的参数 计算每一种的结果
        // 并挑选出最优的解                       
        int err = dubins_words[i](alpha, 
                                  beta, 
                                  d, 
                                  params);
        // 没有出错的话
        if(err == EDUBOK) {
            // 将长度作为cost
            double cost = params[0] + params[1] + params[2];
            // 迭代 找出最小的
            if(cost < best_cost) {
                best_word = i;
                best_cost = cost;
                path->param[0] = params[0];
                path->param[1] = params[1];
                path->param[2] = params[2];
                path->type = i;
            }
        }
    }

    // 每一种情况的都失败
    if(best_word == -1) {
        return EDUBNOPATH;
    }
    // 
    path->type = best_word;
    // 返回正常结果
    return EDUBOK;
}


// dubins初始化
int dubins_init( double q0[3],           // 起点 x y theta
                 double q1[3],           // 终点 x y theta
                 double rho,             // 车辆转弯半径（前进速度除以最大角速度）
                 // path 指针S
                 DubinsPath* path )
{

    int i;
    // 起点到终点的 dx
    double dx = q1[0] - q0[0];
    // 起点到终点的 dy
    double dy = q1[1] - q0[1];
    // 起点 终点距离
    double D = sqrt( dx * dx + dy * dy );
    // 该参数 用于后面dubins曲线长度的计算
    // D / R 
    double d = D / rho;
    // 转弯半径很小 报错 rho值无效
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    // 利用坐标转换 进行归一化计算
    // 反三角计算起点 终点角度
    double theta = mod2pi(atan2( dy, dx ));
    //                    起点theta - 
    double alpha = mod2pi(q0[2] - theta);
    //                    终点theta - 
    double beta  = mod2pi(q1[2] - theta);

    // 将起点信息放入 path成员中
    for( i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    path->rho = rho;

    // 计算归一化后的 dubins曲线
    return dubins_init_normalised( alpha, beta, d, path );
}


// 归一化参数计算 LSL 
int dubins_LSL( double alpha, 
                double beta, 
                double d,   // 
                // 输出每一段的长度结果
                double* outputs )
{
    // 计算 alpha beta 的sin cos值
    // 目的是做一些预计算
    UNPACK_INPUTS(alpha, beta);

    // D/R + sin(alpha) - sin(beta)
    double tmp0 = d + sa - sb;
    // l2 曲线长度平方
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    // 
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    // 临时辅助计算量
    double tmp1 = atan2( (cb-ca), tmp0 );
    // l1需要旋转的角度对应的弧长 转弯半径在函数内部
    double t = mod2pi(-alpha + tmp1 );
    // l2 长度
    double p = sqrt( p_squared );
    // l2需要旋转的角度对应的弧长 转弯半径在函数内部
    double q = mod2pi(beta - tmp1 );

    // 记录每一段长度的结果 l1 l2 l3 以参数形式返回
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}


int dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp0 = d-sa+sb;
    double p_squared = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    double tmp1 = atan2( (ca-cb), tmp0 );
    double t = mod2pi( alpha - tmp1 );
    double p = sqrt( p_squared );
    double q = mod2pi( -beta + tmp1 );
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( p_squared < 0 ) {
        return EDUBNOPATH;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (-ca-cb), (d+sa+sb) ) - atan2(-2.0, p);
    double t    = mod2pi(-alpha + tmp2);
    double q    = mod2pi( -mod2pi(beta) + tmp2 );
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double p_squared = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( p_squared< 0 ) {
        return EDUBNOPATH;
    }
    double p    = sqrt( p_squared );
    double tmp2 = atan2( (ca+cb), (d-sa-sb) ) - atan2(2.0, p);
    double t    = mod2pi(alpha - tmp2);
    double q    = mod2pi(beta - tmp2);
    PACK_OUTPUTS(outputs);
    return EDUBOK;
}

int dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( fabs(tmp_rlr) > 1) {
        return EDUBNOPATH;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_rlr ) );
    double t = mod2pi(alpha - atan2( ca-cb, d-sa+sb ) + mod2pi(p/2.));
    double q = mod2pi(alpha - beta - t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

int dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);
    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;
    if( fabs(tmp_lrl) > 1) {
        return EDUBNOPATH;
    }
    double p = mod2pi( 2*M_PI - acos( tmp_lrl ) );
    double t = mod2pi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.);
    double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
    PACK_OUTPUTS( outputs );
    return EDUBOK;
}

// 求解 dubins曲线的长度
double dubins_path_length( DubinsPath* path )
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    // 总长度 = 长度 * 最小转弯半径？？？
    length = length * path->rho;
    return length;
}

int dubins_path_type( DubinsPath* path ) {
    return path->type;
}


void dubins_segment( double t, double qi[3], double qt[3], int type)
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );

    if( type == L_SEG ) {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else if( type == S_SEG ) {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}


// 对已经生成的dubins曲线 根据t值 进行采样 采样的到的 x y theta 保留到q[3]中
int dubins_path_sample( DubinsPath* path, double t, double q[3] )
{
    // t值超过范围
    if( t < 0 || t >= dubins_path_length(path) ) {
        // error, parameter out of bounds
        return EDUBPARAM;
    }

    // tprime is the normalised variant of the parameter t
    // 这个参数 是什么意思？？？ 
    double tprime = t / path->rho;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component

    // The translated initial configuration
    double qi[3] = { 0, 0, path->qi[2] };

    // Generate the target configuration
    const int* types = DIRDATA[path->type];
    double p1 = path->param[0];
    double p2 = path->param[1];
    double q1[3]; // end-of segment 1
    double q2[3]; // end-of segment 2
    dubins_segment( p1,      qi,    q1, types[0] );
    dubins_segment( p2,      q1,    q2, types[1] );
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] );
    }

    // 将归一化的节点信息 转回到原始值 
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    return 0;
}


int dubins_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize, void* user_data )
{
    double x = 0.0;
    double length = dubins_path_length(path);
    while( x <  length ) {
        double q[3];
        dubins_path_sample( path, x, q );
        int retcode = cb(q, x, user_data);
        if( retcode != 0 ) {
            return retcode;
        }
        x += stepSize;
    }
    return 0;
}

int dubins_path_endpoint( DubinsPath* path, double q[3] )
{
    // TODO - introduce a new constant rather than just using EPSILON
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
{
    // calculate the true parameter
    double tprime = t / path->rho;

    // copy most of the data
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    // fix the parameters
    newpath->param[0] = fmin( path->param[0], tprime );
    newpath->param[1] = fmin( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = fmin( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;
}
}
#endif
