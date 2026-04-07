#include <PDPC/PointCloud/Loader.h>
#include <PDPC/PointCloud/OBJ.h>
#include <PDPC/PointCloud/PLY.h>

#include <PDPC/PointCloud/PointCloud.h>

#include <PDPC/Common/Log.h>
#include "Loader.h"
#include <map>
#include <fstream>
#include<filesystem>
#include <iostream>
#include <random>
namespace pdpc {

bool Loader::Load(const std::string& filename, PointCloud& g, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
    if(ext == "obj")
    {
        return OBJ::load(filename, g, verbose);
    }
    else if(ext == "ply")
    {
        return PLY::load(filename, g, verbose);
    }
    else
    {
        error() << "Missing or unsupported extension for file " << filename;
        return false;
    }
}

bool Loader::Save(const std::string& filename, const PointCloud& g, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
    if(ext == "obj")
    {
        return OBJ::save(filename, g, verbose);
    }
    else if(ext == "ply")
    {
        return PLY::save(filename, g, verbose);
    }
    else if(ext=="vg"){


    }
    else
    {
        return PLY::save(filename + ".ply", g, verbose);
    }
}

bool Loader::saveVG(const std::string &filenaem, const PointCloud &g, const std::vector<int> &labeling,const int threshold, const bool verbose)
{
    //labeling to map
    std::map<int,std::vector<int>> seg_tmp;
    std::map<int,std::vector<int>> seg_final;
    for(int i=0;i<labeling.size();i++){
        if(labeling[i]==-1){
            continue;
        }
        seg_tmp[labeling[i]].push_back(i);

    }
    for (const auto& seg:seg_tmp){
        if(seg.second.size()<threshold){
            continue;
        }
        seg_final[seg.first]=seg.second;
    }
    if(seg_final.size()==0){
        info().iff(true)<<"seg_final.size()=0";
    }
    std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);
	std::string savePath = filenaem;


	std::ofstream output(savePath.c_str());
	if (output.fail()) {
		std::cerr << "could not open file\'" << savePath << "\'" << std::endl;
		return false;
	}
	output.precision(16);
	const int pointsNum = g.size();
	output << "num_points: " << g.size() << std::endl;
	for (std::size_t i = 0; i <g.size(); ++i) {
		output << g.point(i).x() << " " << g.point(i).y() << " " << g.point(i).z() << std::endl;
	}
	output << "num_colors: 0" << std::endl;

	if (g.has_normals()) {
		output << "num_normals: " << g.size() << std::endl;
		for (std::size_t i = 0; i < g.size(); ++i) {
			output << g.normal(i).x() << " " << g.normal(i).y() << " " << g.normal(i).z() << std::endl;
		}
	}
	else {
		std::cerr << "normals size is not equal to points size!" << std::endl;
		output << "num_normals: 0" << std::endl;
	}
	output << "num_groups: " << seg_final.size() << std::endl;
	for (const auto& seg : seg_final) {
		output << "group_type: 0" << std::endl;
		output << "num_group_parameters: 4" << std::endl;
		const auto& plane = fitPlane(g,seg.second);
		output << "group_parameters: " << plane.a() << " " << plane.b() << " " << plane.c() << " " << plane.d() << std::endl;
		output << "group_label: unknown" << std::endl;
		output << "group_color: " << dis(gen) << " " << dis(gen) << " " << dis(gen) << std::endl;
		output << "group_num_points: " << seg.second.size() << std::endl;
		for (const auto& idx : seg.second) {
			output << idx << " ";
		}
		output << std::endl;
		output << "num_children: 0" << std::endl;
	}
	output.close();
    info().iff(true)<<"save vg to "<<filenaem;
    return true;
}
bool Loader::saveVGFast(const std::string &filenaem, const PointCloud &g, const std::vector<int> &labeling, const int threshold, const bool verbose)
{
    // labeling to map
    std::unordered_map<int, std::vector<int>> seg_tmp;
    std::unordered_map<int, std::vector<int>> seg_final;

    for (size_t i = 0; i < labeling.size(); i++) {
        if (labeling[i] == -1) continue;
        seg_tmp[labeling[i]].push_back((int)i);
    }

    for (const auto &seg : seg_tmp) {
        if (seg.second.size() < (size_t)threshold) continue;
        seg_final[seg.first] = seg.second;
    }

    if (seg_final.empty()) {
        info().iff(true) << "seg_final.size()=0";
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    std::ofstream output(filenaem.c_str(), std::ios::out | std::ios::binary);
    if (!output) {
        std::cerr << "could not open file '" << filenaem << "'" << std::endl;
        return false;
    }

    // Use a larger output buffer (1 MB)
    static char buffer[1 << 20];
    output.rdbuf()->pubsetbuf(buffer, sizeof(buffer));

    std::ostringstream oss;
    oss.precision(16);
    oss.setf(std::ios::fixed, std::ios::floatfield);

    const int pointsNum = (int)g.size();
    oss << "num_points: " << pointsNum << '\n';

    // Write point coordinates
    for (size_t i = 0; i < g.size(); ++i) {
        const auto &p = g.point(i);
        oss << p.x() << " " << p.y() << " " << p.z() << '\n';
    }

    oss << "num_colors: 0\n";

    // Write normals
    if (g.has_normals()) {
        oss << "num_normals: " << g.size() << '\n';
        for (size_t i = 0; i < g.size(); ++i) {
            const auto &n = g.normal(i);
            oss << n.x() << " " << n.y() << " " << n.z() << '\n';
        }
    } else {
        std::cerr << "normals size is not equal to points size!" << std::endl;
        oss << "num_normals: 0\n";
    }

    // Write groups
    oss << "num_groups: " << seg_final.size() << '\n';
    for (const auto &seg : seg_final) {
        oss << "group_type: 0\n";
        oss << "num_group_parameters: 4\n";

        const auto &plane = fitPlane(g, seg.second);
        oss << "group_parameters: " << plane.a() << " "
            << plane.b() << " "
            << plane.c() << " "
            << plane.d() << '\n';

        oss << "group_label: unknown\n";
        oss << "group_color: " << dis(gen) << " "
            << dis(gen) << " "
            << dis(gen) << '\n';

        oss << "group_num_points: " << seg.second.size() << '\n';
        for (const auto &idx : seg.second) {
            oss << idx << " ";
        }
        oss << '\n';
        oss << "num_children: 0\n";
    }

    // Flush the accumulated content in one write
    output << oss.str();
    output.close();

    info().iff(true) << "save vg to " << filenaem;
    return true;
}
Plane_3 Loader::fitPlane(const PointCloud &g, const std::vector<int> &pointsIndx)
{
	std::vector<Point_3> pointsFitting;
	for (const auto& i : pointsIndx) {
		pointsFitting.push_back(Point_3(g.point(i).x(), g.point(i).y(), g.point(i).z()));
	}
	Plane_3 plane;
	CGAL::linear_least_squares_fitting_3(pointsFitting.begin(), pointsFitting.end(), plane, CGAL::Dimension_tag<0>());

	return plane;
}
} // namespace pdpc
