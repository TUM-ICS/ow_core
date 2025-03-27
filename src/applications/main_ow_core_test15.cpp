#include <ow_core/types.h>
#include <ow_core/common/parameter.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");

  std::string ns = "/test_ik/";

  std::vector<std::string> sv = {
    "/body/arms/left/base",
    "/body/arms/left/ee",
    "/body/arms/left/name",
    "/body/arms/right/base"
  };

  std::vector<std::string> dv = {
    "/solver/alpha",
    "/solver/beta",
    "/solver/k_null"
  };

  ow::Parameter parameter(ns);
  parameter.add<std::string>(sv);
  parameter.add<double>(dv);
  parameter.add<double>("/solver/gamma", 100);

  if(!parameter.load())
  {
    ROS_ERROR("Error in loading params");
  }

  double v_val = parameter.get<double>("/solver/gamma");
  std::cout << "got v_val=" << v_val << std::endl;

  //Eigen::Vector4d vec = parameter.get<ow::MatrixX>(1);
  //std::cout << "vec=" << vec << std::endl;

  /*if(ns.back() != '/')
    ns.push_back('/');
  if(ns.front() != '/')
    ns.insert(ns.begin(),'/');

  std::cout << "namespace=" << ns << std::endl;

  std::vector<std::string> names;
  ros::param::getParamNames(names);

  size_t pos;
  std::string var_name, path_name;
  for(int i = 0; i < names.size(); ++i)
  {
    const std::string& name = names[i];

    // check if inside namespace
    pos = name.find(ns);
    if(pos == std::string::npos || pos != 0) 
    {
      continue;
    }
    path_name = name.substr(pos + ns.size(), name.size());
    std::cout << path_name << std::endl;

    // get the variable name
    pos = name.find_last_of('/');
    if(pos == std::string::npos)
    {
      continue;
    }

    var_name = name.substr(pos+1, name.size());
    std::cout << var_name << std::endl;
  }*/


  return 0;
}

/*

  int load()
  {
    std::vector<std::string> names;
    ros::param::getParamNames(names);

    ROS_ERROR("ERROR!!!");
    return T();
  }
  
    bool extract(
    const std::string& name, 
    std::string& param, 
    std::string& path) const
  {
    std::ostringstream out;
    for(size_t i = 0; i < entries_.size(); ++i)
    {
      out << "elem[" << i << "] = [" << entries_[i]->toString() << "]\n";
    }
    return out.str();
  }
  
    int load()
  {
    std::vector<std::string> names;
    ros::param::getParamNames(names);

    std::string parameter, identifier; 
    for(size_t i = 0; i < names.size(); ++i)
    {
      if(extract(names[i], parameter, identifier))
      {
        std::cout << "parameter=" << parameter << std::endl;
        std::cout << "identifier=" << identifier << std::endl;
      }
    }
  }

  */