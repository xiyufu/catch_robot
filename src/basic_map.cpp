#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>

void ReadMapFile(std::vector<std::vector<double>>& envmap, const std::string file_name){
    std::string package_path = ros::package::getPath("catch_point_robot");
    std::string full_name = package_path + "/" + file_name;
    std::ifstream file_map(full_name);
    std::string line;
    
    while(getline(file_map, line)){
        std::vector<double> row;
        double grid;
        std::string grid_str;

        for (int i = 0; i < line.size()+1; ++i){
            if (line[i] == ' ' || line[i] == '\0'){
                if (!grid_str.empty()){
                    grid = std::stod(grid_str);
                    row.push_back(grid);
                    grid_str.clear();
                }
                continue;
            }
            grid_str += line[i];
        }
        if (!row.empty()){
            envmap.push_back(row);
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "basic_maps");
    if (argc < 2){
        ROS_WARN("Please specify the name of file");
        return -1;
    }
    std::vector<std::vector<double>> envmap;
    ReadMapFile(envmap, argv[1]);

    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("basic_map", 1);

    while (ros::ok()){
        nav_msgs::OccupancyGrid bit_map;
        bit_map.header.frame_id = "map";
        bit_map.header.stamp = ros::Time::now();

        bit_map.info.origin.position.x = 0;
        bit_map.info.origin.position.y = 0;
        bit_map.info.origin.position.z = 0;
        bit_map.info.origin.orientation.x = 0;
        bit_map.info.origin.orientation.y = 0;
        bit_map.info.origin.orientation.z = 0;
        bit_map.info.origin.orientation.w = 1.0;

        bit_map.info.map_load_time = ros::Time::now();
        bit_map.info.resolution = 1;
        bit_map.info.height = envmap.size();
        bit_map.info.width = envmap[0].size();

        for (int i = 0; i < bit_map.info.height; ++i){
            for (int j = 0; j < bit_map.info.width; ++j){
                // int id_serial = i * bit_map.info.width + j;
                if (envmap[i].size() < bit_map.info.width){
                    ROS_WARN_ONCE("The %dth row of the map is shorter than the width of the map", i);
                    continue;
                }
                if (envmap[i][j] == 1){
                    envmap[i][j] *= 100;
                }
                bit_map.data.push_back(envmap[i][j]);
            }
        }

        while (map_pub.getNumSubscribers() < 1){
            if (!ros::ok()){
                return 0;
            }
            ROS_WARN_ONCE("Please subscribe to /basic_map");
            sleep(1);
        }
        map_pub.publish(bit_map);
        ROS_INFO_ONCE("Publishing");
        loop_rate.sleep();
    }
    return 0;
}