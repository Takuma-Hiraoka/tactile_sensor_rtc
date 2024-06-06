#include <iostream>
#include <boost/program_options.hpp>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/YAMLWriter>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshFilter>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

class TactileSensor
{
 public:
  std::string name;
  std::string linkName;
  cnoid::Vector3f p; // リンク座標系
  cnoid::Matrix3f R; // リンク座標系．zがリンク内側方向
};


int main(int argc, char *argv[]){
  boost::program_options::options_description opt("Options");
  opt.add_options()
    ("help,h", "read model file(.wrl) and write each tactile sensor configuration(.yaml). config_writer -i [file path] -o [file path]")
    ("input,i", boost::program_options::value<std::string>(),"input file path")
    ("output,o", boost::program_options::value<std::string>(),"output file path")
    ("resolution,r", boost::program_options::value<float>()->default_value(0.01),"resolution");
  boost::program_options::variables_map argmap;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opt), argmap );
  boost::program_options::notify( argmap );

  if( argmap.count( "help" ) ){
    std::cerr << opt << std::endl;
    return 1;
  }
  if( argmap.count( "input" )==0 ){
    std::cerr << "input file is not supplied" << std::endl;
    return 1;
  }
  std::string inputFilePath = argmap["input"].as<std::string>();
  if( argmap.count( "output" )==0 ){
    std::cerr << "output file is not supplied" << std::endl;
    return 1;
  }
  std::string outputFilePath = argmap["output"].as<std::string>();
  float resolution = argmap["resolution"].as<float>();;
  if(!(resolution > 0.0)){
    std::cerr << "resolution <= 0.0" << std::endl;
    return 1;
  }
  float normalAngle = M_PI/3;

  cnoid::BodyPtr robot;
  {
    cnoid::BodyLoader bodyLoader;
    robot = bodyLoader.load(inputFilePath);
    if(!robot){
      std::cerr << "failed  to load " << inputFilePath << std::endl;
      return 1;
    }
  }

  std::vector<TactileSensor> sensors;
  cnoid::MeshExtractor meshExtractor;
  cnoid::MeshFilter meshFilter;
  for(int i=0;i<robot->numLinks();i++){
    if(robot->link(i)->collisionShape()->numChildObjects()==0) continue;
    std::string linkName = robot->link(i)->collisionShape()->child(0)->name();
    if(linkName == "") linkName = robot->link(i)->name();

    cnoid::SgMeshPtr mesh = meshExtractor.integrate(robot->link(i)->collisionShape());
    if(!mesh || (mesh->numTriangles() == 0)) continue;
    meshFilter.generateNormals(mesh,M_PI,true);

    std::vector<std::vector<std::vector<std::vector<TactileSensor> > > > bin;
    mesh->updateBoundingBox();
    cnoid::BoundingBoxf bbx = mesh->boundingBox();
    cnoid::Vector3f bbxSize = bbx.max() - bbx.min();
    bin.resize(int(bbxSize[0]/resolution)+1);
    for(int x=0;x<bin.size();x++){
      bin[x].resize(int(bbxSize[1]/resolution)+1);
      for(int y=0;y<bin[x].size();y++){
        bin[x][y].resize(int(bbxSize[2]/resolution)+1);
      }
    }

    for(int j=0;j<mesh->numTriangles();j++){
      cnoid::Vector3f v0 = mesh->vertices()->at(mesh->triangle(j)[0]);
      cnoid::Vector3f v1 = mesh->vertices()->at(mesh->triangle(j)[1]);
      cnoid::Vector3f v2 = mesh->vertices()->at(mesh->triangle(j)[2]);
      cnoid::Vector3f normal = - mesh->normals()->at(mesh->normalIndices()[j*3]); // linkの内側に向かう方向

      float l1 = (v1 - v0).norm();
      float l2 = (v2 - v0).norm();
      cnoid::Vector3f d1 = (v1 - v0).normalized();
      cnoid::Vector3f d2 = (v2 - v0).normalized();

      for(float m=0;;){
        float n_max = (l1==0)? l2 : l2*(1-m/l1);
        for(float n=0;;){
          cnoid::Vector3f v = v0 + d1 * m + d2 * n;
          int x = int((v[0] - bbx.min()[0])/resolution);
          int y = int((v[1] - bbx.min()[1])/resolution);
          int z = int((v[2] - bbx.min()[2])/resolution);
          bool exists = false;
          for(int s=0;s<bin[x][y][z].size();s++){
            if(normalAngle >= std::acos(std::min(1.0f,(std::max(-1.0f,normal.dot(bin[x][y][z][s].R * cnoid::Vector3f::UnitZ())))))){
              exists = true;
              break;
            }
          }
          if(!exists){
            TactileSensor sensor;
            sensor.linkName = linkName;
            sensor.p = v;
            cnoid::Vector3f z_axis = normal;
            cnoid::Vector3f x_axis = (z_axis==cnoid::Vector3f::UnitY()) ? cnoid::Vector3f::UnitZ() : cnoid::Vector3f::UnitY().cross(z_axis);
            cnoid::Vector3f y_axis = z_axis.cross(x_axis);
            sensor.R.col(0) = x_axis.normalized(); sensor.R.col(1) = y_axis.normalized(); sensor.R.col(2) = z_axis.normalized();
            bin[x][y][z].push_back(sensor);
          }

          if(n>= n_max) break;
          else n = std::min(n+resolution, n_max);
        }

        if(m>=l1) break;
        else m = std::min(m+resolution, l1);
      }
    }

    int id = 0;
    for(int x=0;x<bin.size();x++){
      for(int y=0;y<bin[x].size();y++){
        for(int z=0;z<bin[x][y].size();z++){
          for(int s=0;s<bin[x][y][z].size();s++){
            bin[x][y][z][s].name = bin[x][y][z][s].linkName + "ts" + std::to_string(id);
            sensors.push_back(bin[x][y][z][s]);
            id++;
          }
        }
      }
    }
  }


  cnoid::YAMLWriter writer(outputFilePath);
  writer.startMapping();
  writer.putKey("tactile_sensor");
  writer.startListing();
  for (int i=0; i<sensors.size(); i++) {
    writer.startMapping();
    writer.startListing();
    {
      writer.putKeyValue("name",sensors[i].name);
      writer.putKeyValue("link",sensors[i].linkName);
      writer.putKey("translation");
      writer.startFlowStyleListing();
      for (int j=0; j<3; j++) writer.putScalar(sensors[i].p[j]);
      writer.endListing();
      writer.putKey("rotation");
      writer.startFlowStyleListing();
      cnoid::AngleAxisf R(sensors[i].R);
      writer.putScalar(R.axis()[0]); writer.putScalar(R.axis()[1]); writer.putScalar(R.axis()[2]); writer.putScalar(R.angle());
      writer.endListing();
      writer.putKeyValue("radius",resolution);
    }
    writer.endListing();
    writer.endMapping();
  }
  writer.endListing();
  writer.endMapping();

  return 0;
}
