#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);//键id为int型，值positions是Vector3f数组

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

/*
**在Screen Space中判断坐标点(x,y)是否在三角形内
*/
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //获取三角形的三个顶点
    const Vector3f& A = _v[0];
    const Vector3f& B = _v[1];
    const Vector3f& C = _v[2];

    //顺序连接顶点成线
    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;

    //使用叉乘判断像素点(x,y,1)是否在三角形ABC内
    Vector3f P(x, y, 1.0);//齐次坐标，位置点齐次坐标项取1(向量齐次坐标项取0)
    Vector3f AP = P - A;//顺序连线
    Vector3f BP = P - B;
    Vector3f CP = P - C;
    Vector3f res_ABP = AB.cross(AP);//做叉乘，结果为向量
    Vector3f res_BCP = BC.cross(BP);
    Vector3f res_CAP = CA.cross(CP);

    //如果三个叉乘结果向量的Z坐标值同为正或同为负，说明像素点(x,y,1)在三角形ABC内
    return (res_ABP.z() > 0  &&  res_BCP.z() > 0  &&  res_CAP.z() > 0) || (res_ABP.z() < 0  &&  res_BCP.z() < 0  &&  res_CAP.z() < 0);
}

/*
**像素点(x,y)在二维平面三角形v中的重心坐标
*/
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

//
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    /*
    **通过map和引用，将整个顶点数组，三角形序列数组，顶点颜色数组全部传入进来
    */
    auto& buf = pos_buf[pos_buffer.pos_id];//buf类型为pos_buf的值类型，即：坐标数组(元素是三维坐标)
    auto& ind = ind_buf[ind_buffer.ind_id];//ind类型同上
    auto& col = col_buf[col_buffer.col_id];//col类型同上

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)//i类型为ind的元素类型，即三维坐标Vector3i
    {
        Triangle t;

        /*
        **对所有顶点坐标进行MVP变换
        */
        Eigen::Vector4f v[] = {//坐标数组
                mvp * to_vec4(buf[i[0]], 1.0f),//buf[]类型为三维坐标Vector3f，i[]类型为int型。综上，将buf数组中第i[]个元素(三维坐标)转化为四维坐标，并进行MVP变换
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };//元素为MVP变换后的顶点坐标值,，此时位于Clip Space
        
        /*
        **顶点坐标齐次坐标项令为1，其余坐标项同等变化
        */
        for (auto& vec : v) {//vec类型为四维坐标Vector4f
            vec /= vec.w();
        }

        /*
        **Viewport transformation
        */
        for (auto & vert : v)//vert类型为四维坐标Vector4f
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        /*
        **设置三角形顶点坐标(screen space)
        */
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());//v[]类型为四维坐标Vector4f
            //t.setVertex(i, v[i].head<3>());
            //t.setVertex(i, v[i].head<3>());
        }

        /*
        **设置三角形中顶点的各自颜色值
        */
        auto col_A = col[i[0]];//col[]类型为三维坐标Vector3f，i[]类型为int型
        auto col_B = col[i[1]];
        auto col_C = col[i[2]];

        /*
        **对三角形中三个顶点确定颜色值
        */
        t.setColor(0, col_A[0], col_A[1], col_A[2]);
        t.setColor(1, col_B[0], col_B[1], col_B[2]);
        t.setColor(2, col_C[0], col_C[1], col_C[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();//v是包含三个元素的数组，元素类型为Vector4f
    
    //在Screen Space中确定AABB轴对齐包围盒
    float x_min=std::min(std::min(v[0][0], v[1][0]), v[2][0]);
    float x_max=std::max(std::max(v[0][0], v[1][0]), v[2][0]);
    float y_min=std::min(std::min(v[0][1], v[1][1]), v[2][1]);
    float y_max=std::max(std::max(v[0][1], v[1][1]), v[2][1]);

    // anti-alising
    bool MSAA4X=true; 

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    if(!MSAA4X)
    {
        // without anti-alising
        for(int x=(int)x_min; x<=(int)x_max; x++)
        {
            for(int y=(int)y_min; y<=(int)y_max; y++)//以递增方式遍历整个AABB中的所有坐标点
            {
                //判断坐标点(x,y)是否在形参t(三角形)的内部
                if(!insideTriangle((float)x,(float)y,t.v))    continue;
                // get z value--depth
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);//计算坐标点(x,y)在形参t(三角形)中的重心坐标
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());//利用重心坐标插值计算齐次项的值的倒数
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();//插值计算z分项值
                z_interpolated *= w_reciprocal;//对z分项值再进行齐次项处理，结果作为zbuffer值

                // compare the current depth with the value in depth buffer
                if(depth_buf[get_index(x,y)]>z_interpolated)// note: we use get_index to get the index of current point in depth buffer
                {
                    // we have to update this pixel
                    depth_buf[get_index(x,y)]=z_interpolated; // update depth buffer
                    // assign color to this pixel
                    set_pixel(Vector3f(x,y,z_interpolated), t.getColor());
                }
            }
        }
    }
    else
    {
        for(int x=(int)x_min; x<=(int)x_max; x++)
        {
            for(int y=(int)y_min; y<=(int)y_max; y++)//以递增方式遍历整个AABB中的所有坐标点
            {
                float min_depth = FLT_MAX;//记录像素内四个不同采样点里面最小的deepth，以作为最终deepth
                int count = 0;//记录像素内位于三角形内部的采样点的个数
                std::vector<std::vector<float>> sampled_points{{0.25,0.25},{0.25,0.75},{0.75,0.25},{0.75,0.75}};//像素内取四个采样点
                for(int i=0; i<4; i++)//遍历每个采样点
                {
                    if(insideTriangle(float(x)+sampled_points[i][0], float(y)+sampled_points[i][1], t.v))//判断采样点是否在三角形内
                    {
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        min_depth=std::min(min_depth, z_interpolated);//保存四个采样点中最小的z_interpolated
                        count += 1;
                    }
                }
                if(count>0 && depth_buf[get_index(x,y)]>min_depth)
                {
                    //更新像素点(x,y)对应的z-buffer值
                    depth_buf[get_index(x,y)]=min_depth;
                    // note: the color should be changed too
                    set_pixel(Vector3f(x,y,min_depth), t.getColor()*count/4.0+frame_buf[get_index(x,y)]*(4-count)/4.0); // frame_buf contains the current color
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

/*
**Screen space是二维平面，每个点坐标为(x,y)，但存储是则按一维数组存储，故每个点在一维数组中可通过索引值来直接读取
*/
int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on