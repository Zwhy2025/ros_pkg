/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "urdf/model.h"

// 包括普通URDF文件的默认解析器；
// 其他解析器通过插件加载（如果可用）
#include <urdf_parser/urdf_parser.h>
#include <urdf_parser_plugin/parser.h>
#include <pluginlib/class_loader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

namespace urdf
{

    static bool IsColladaData(const std::string & data)
    {
        return data.find("<COLLADA") != std::string::npos; // 检查数据是否为COLLADA格式
    }

    bool Model::initFile(const std::string & filename)
    {
        // 从文件中获取整个XML字符串
        std::string xml_string;
        std::fstream xml_file(filename.c_str(), std::fstream::in);
        if (xml_file.is_open()) {
            while (xml_file.good()) {
                std::string line;
                std::getline(xml_file, line);
                xml_string += (line + "\n");
            }
            xml_file.close();
            return Model::initString(xml_string); // 使用XML字符串初始化模型
        } else {
            ROS_ERROR("Could not open file [%s] for parsing.", filename.c_str());
            return false;
        }
    }

    bool Model::initParam(const std::string & param)
    {
        ROS_INFO("param: %s ", param.c_str());
        return initParamWithNodeHandle(param, ros::NodeHandle()); // 使用节点句柄初始化参数
    }

    bool Model::initParamWithNodeHandle(const std::string & param, const ros::NodeHandle & nh)
    {
        std::string xml_string;

        // 从参数服务器获取机器人描述的位置
        std::string full_param;
        if (!nh.searchParam(param, full_param)) {
            ROS_ERROR("Could not find parameter %s on parameter server", param.c_str());
            return false;
        }

        // 从参数服务器读取机器人描述
        if (!nh.getParam(full_param, xml_string)) {
            ROS_ERROR("Could not read parameter %s on parameter server", full_param.c_str());
            return false;
        }

        ROS_INFO("robot description file: %s ", xml_string.c_str());
        return Model::initString(xml_string); // 使用XML字符串初始化模型
    }

    bool Model::initXml(TiXmlDocument * xml_doc)
    {
        // 使用TinyXML文档初始化模型
        if (!xml_doc) {
            ROS_ERROR("Could not parse the xml document");
            return false;
        }

        std::stringstream ss;
        ss << *xml_doc;

        return Model::initString(ss.str());
    }

    bool Model::initXml(TiXmlElement * robot_xml)
    {
        // 使用TinyXML元素初始化模型
        if (!robot_xml) {
            ROS_ERROR("Could not parse the xml element");
            return false;
        }

        std::stringstream ss;
        ss << (*robot_xml);

        return Model::initString(ss.str());
    }

    bool Model::initString(const std::string & xml_string)
    {
        urdf::ModelInterfaceSharedPtr model;

        // COLLADA兼容性处理
        if (IsColladaData(xml_string)) {
            ROS_DEBUG("Parsing robot collada xml string");

            static boost::mutex PARSER_PLUGIN_LOCK;
            static boost::scoped_ptr<pluginlib::ClassLoader<urdf::URDFParser>> PARSER_PLUGIN_LOADER;
            boost::mutex::scoped_lock _(PARSER_PLUGIN_LOCK);

            try {
                if (!PARSER_PLUGIN_LOADER)
                    PARSER_PLUGIN_LOADER.reset(new pluginlib::ClassLoader<urdf::URDFParser>("urdf_parser_plugin", "urdf::URDFParser"));
                const std::vector<std::string> &classes = PARSER_PLUGIN_LOADER->getDeclaredClasses();
                bool found = false;
                for (std::size_t i = 0; i < classes.size(); ++i)
                    if (classes[i].find("urdf/ColladaURDFParser") != std::string::npos) {
                        boost::shared_ptr<urdf::URDFParser> instance = PARSER_PLUGIN_LOADER->createInstance(classes[i]);
                        if (instance)
                            model = instance->parse(xml_string);
                        found = true;
                        break;
                    }
                if (!found)
                    ROS_ERROR_STREAM("No URDF parser plugin found for Collada files. Did you install the corresponding package?");
            } catch (pluginlib::PluginlibException &ex) {
                ROS_ERROR_STREAM("Exception while creating planning plugin loader " << ex.what() << ". Will not parse Collada file.");
            }
        } else {
            ROS_INFO("Parsing robot urdf xml string");
            model = parseURDF(xml_string); // 使用URDF解析器解析XML字符串
        }

        // 将模型数据复制到此对象中
        if (model) {
            this->links_ = model->links_;
            this->joints_ = model->joints_;
            this->materials_ = model->materials_;
            this->name_ = model->name_;
            this->root_link_ = model->root_link_;
            return true;
        }
        return false;
    }
} // namespace urdf

