

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <cstdint>


#include <iostream>
#include <list>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

typedef CGAL::Simple_cartesian<double> K;

typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef K::Direction_3 Direction;
typedef K::Vector_3 Vector3;

typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;



bool _MakeAABB(std::list<Triangle> &triangles, FILE *fp)
{
  if(fgetc(fp)!='A')return false;
  if(fgetc(fp)!='A')return false;
  if(fgetc(fp)!='B')return false;
  if(fgetc(fp)!='B')return false;
  if(fgetc(fp)!='T')return false;
  if(fgetc(fp)!=0x0)return false;
  if(fgetc(fp)!=0x0)return false;
  
  std::vector<Point> pt;
  float x,y,z;
  
  while(1)
  {
    pt.clear();
    for(int pi=0;pi<3;pi++)
    {
      if(fread(&x, sizeof(float), 1, fp)!=1)return true;
      if(fread(&y, sizeof(float), 1, fp)!=1)return false;
      if(fread(&z, sizeof(float), 1, fp)!=1)return false;
      pt.push_back(Point(x,y,z));
    }
    triangles.push_back(Triangle(pt[0], pt[1], pt[2]));
  }
  return true;
}
bool MakeAABB(std::list<Triangle> &triangles, const char *path)
{
  FILE *fp = fopen(path, "rb");
  if(fp==NULL)return false;
  bool bRet = _MakeAABB(triangles, fp);
  fclose(fp);
  return bRet;
}

bool _ReadMoveVertex(std::vector<Point> &pt, std::vector<uint64_t> &oivi, FILE *fp)
{
  if(fgetc(fp)!='A')return false;
  if(fgetc(fp)!='A')return false;
  if(fgetc(fp)!='B')return false;
  if(fgetc(fp)!='B')return false;
  if(fgetc(fp)!='V')return false;
  if(fgetc(fp)!=0x0)return false;
  if(fgetc(fp)!=0x0)return false;
  float x,y,z;
  uint64_t _oivi;
  
  while(1)
  {
    for(int pi=0;pi<3;pi++)
    {
      if(fread(&_oivi, sizeof(uint64_t), 1, fp)!=1)return true;
      oivi.push_back(_oivi);
      if(fread(&x, sizeof(float), 1, fp)!=1)return false;
      if(fread(&y, sizeof(float), 1, fp)!=1)return false;
      if(fread(&z, sizeof(float), 1, fp)!=1)return false;
      pt.push_back(Point(x,y,z));
    }
  }
  return true;
}
bool ReadMoveVertex(std::vector<Point> &pt, std::vector<uint64_t> &oivi, const char *path)
{
  FILE *fp = fopen(path, "rb");
  if(fp==NULL)return false;
  bool bRet = _ReadMoveVertex(pt, oivi, fp);
  fclose(fp);
  return bRet;
}

bool MoveClosestPoint(Tree &tree, std::vector<Point> &pt, std::vector<uint64_t> &oivi, FILE *fp)
{
  if(fputc('A', fp)==EOF)return false;
  if(fputc('A', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('V', fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;
  
  float x, y, z;
  
  int numPt = pt.size();
  for(int i=0;i<numPt;i++)
  {
    if(fwrite(&(oivi[i]), sizeof(uint64_t), 1, fp)!=1)return false;
    Point closest_point = tree.closest_point(pt[i]);
    x = closest_point[0];
    y = closest_point[1];
    z = closest_point[2];
    if(fwrite(&x, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&y, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&z, sizeof(float), 1, fp)!=1)return false;
  }
  return true;
}

bool MoveXYZAxis(Tree &tree, std::vector<Point> &pt, std::vector<uint64_t> &oivi, int modexyz, FILE *fp)
{
  if(fputc('A', fp)==EOF)return false;
  if(fputc('A', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('V', fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;
  
  Direction d1, d2;
  switch(modexyz)
  {
  case 0:
    d1 = Direction(Vector3(1.0, 0.0, 0.0));
    d2 = Direction(Vector3(-1.0, 0.0, 0.0));
    break;
  case 1:
    d1 = Direction(Vector3(0.0, 1.0, 0.0));
    d2 = Direction(Vector3(0.0, -1.0, 0.0));
    break;
  default:
    d1 = Direction(Vector3(0.0, 0.0, 1.0));
    d2 = Direction(Vector3(0.0, 0.0, -1.0));
    break;
  }
  
  float x, y, z;
  
  int numPt = pt.size();
  for(int i=0;i<numPt;i++)
  {
    Ray r1(pt[i], d1);
    Ray r2(pt[i], d2);
    Ray_intersection intersection1 = tree.first_intersection(r1);
    Ray_intersection intersection2 = tree.first_intersection(r2);
    Point *p1 = NULL, *p2 = NULL;
    double len1 = DBL_MAX, len2 = DBL_MAX;
    if(intersection1){
      if(boost::get<Point>(&(intersection1->first))){
        p1 =  boost::get<Point>(&(intersection1->first) );
        Vector3 v1(pt[i], *p1);
        len1 = v1.squared_length();
      }
    }
    if(intersection2){
      if(boost::get<Point>(&(intersection2->first))){
        p2 =  boost::get<Point>(&(intersection2->first) );
        Vector3 v2(pt[i], *p2);
        len2 = v2.squared_length();
      }
    }
    if(p1==NULL && p2==NULL)continue;

    Point *pz = (len1<len2)?p1:p2;
    x = (*pz)[0];
    y = (*pz)[1];
    z = (*pz)[2];
    if(fwrite(&(oivi[i]), sizeof(uint64_t), 1, fp)!=1)return false;
    if(fwrite(&x, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&y, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&z, sizeof(float), 1, fp)!=1)return false;
  }
  return true;
}

bool MoveCameraAxis(Tree &tree, std::vector<Point> &pt, std::vector<uint64_t> &oivi, Point &p_camera, FILE *fp)
{
  if(fputc('A', fp)==EOF)return false;
  if(fputc('A', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('B', fp)==EOF)return false;
  if(fputc('V', fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;
  if(fputc(0x0, fp)==EOF)return false;

  float x, y, z;
  
  int numPt = pt.size();
  for(int i=0;i<numPt;i++)
  {
    Vector3 v1(p_camera, pt[i]);
    Vector3 v2(pt[i], p_camera);
    
    Ray r1(pt[i], v1.direction());
    Ray r2(pt[i], v2.direction());
    Ray_intersection intersection1 = tree.first_intersection(r1);
    Ray_intersection intersection2 = tree.first_intersection(r2);
    Point *p1 = NULL, *p2 = NULL;
    double len1 = DBL_MAX, len2 = DBL_MAX;
    if(intersection1){
      if(boost::get<Point>(&(intersection1->first))){
        p1 =  boost::get<Point>(&(intersection1->first) );
        Vector3 v1(pt[i], *p1);
        len1 = v1.squared_length();
      }
    }
    if(intersection2){
      if(boost::get<Point>(&(intersection2->first))){
        p2 =  boost::get<Point>(&(intersection2->first) );
        Vector3 v2(pt[i], *p2);
        len2 = v2.squared_length();
      }
    }
    if(p1==NULL && p2==NULL)continue;

    Point *pz = (len1<len2)?p1:p2;
    x = (*pz)[0];
    y = (*pz)[1];
    z = (*pz)[2];
    if(fwrite(&(oivi[i]), sizeof(uint64_t), 1, fp)!=1)return false;
    if(fwrite(&x, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&y, sizeof(float), 1, fp)!=1)return false;
    if(fwrite(&z, sizeof(float), 1, fp)!=1)return false;
  }
  return true;
}

bool ReadCameraPos(std::vector<float> &_cameraPos, const char *path)
{
  FILE *fp = fopen(path, "rb");
  if(fp==NULL)return false;
  _cameraPos.resize(3);
  int readElement = fread(&(_cameraPos[0]), sizeof(float), 1, fp);
  readElement += fread(&(_cameraPos[1]), sizeof(float), 1, fp);
  readElement += fread(&(_cameraPos[2]), sizeof(float), 1, fp);
  return readElement==3;
}

#include <boost/program_options.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  
  int mode = 0;
  std::vector<float> _cameraPos;
  
  po::options_description desc("options:");
  desc.add_options()
    ("help", "help")
    ("mode,m", po::value<int>(&mode)->default_value(1), "変換モード:\n\t0: 最短面上に吸着\n\t1/2/3: X/Y/Z軸を制限して最短面上に吸着\n\t4: カメラ方向に移動を制限して最短面上に吸着")
    ("in,i", po::value< std::string >(), "input file")
    ("target,t", po::value< std::string >(), "吸着先メッシュ入力")
    ("out,o", po::value< std::string >(), "output file")
    //("camera,c", po::value<std::vector<double> >(&_cameraPos)->multitoken(), "変換モード4のみ使用。カメラ位置 (例: -c 0.0 10.0 5.0)")
    ("camera,c", po::value< std::string >(), "変換モード4のみ使用。カメラ位置")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }
  
  if (!vm.count("in"))
  {
    std::cout << "入力ファイルが指定されていません\n";
    return -1;
  }
  
  if (!vm.count("target"))
  {
    std::cout << "吸着先メッシュが指定されていません\n";
    return -1;
  }
  
  if (!vm.count("out"))
  {
    std::cout << "出力ファイルが指定されていません\n";
    return -1;
  }
  
  
  
  
  bool bRet = false;
  
  std::list<Triangle> triangles;
  bRet = MakeAABB(triangles, vm["target"].as<std::string>().c_str());
  if(!bRet)return -1;
  Tree tree(triangles.begin(),triangles.end());
  
  std::vector<Point> pt;
  std::vector<uint64_t> oivi;
  
  bRet = ReadMoveVertex(pt, oivi, vm["in"].as<std::string>().c_str());
  if(!bRet)return -1;
  
  FILE *fp = fopen(vm["out"].as<std::string>().c_str(), "wb");
  if(fp==NULL)return -1;
  
  switch(mode)
  {
  case 0:
  default:
    bRet = MoveClosestPoint(tree, pt, oivi, fp);
    break;
  case 1:
  case 2:
  case 3:
    bRet = MoveXYZAxis(tree, pt, oivi, mode-1, fp);
    break;
  case 4:
    if (!vm.count("camera"))
    {
      std::cout << "カメラ位置ファイルが指定されていません\n";
      return -1;
    }
    bRet = ReadCameraPos(_cameraPos, vm["camera"].as<std::string>().c_str());

    if (bRet)
    {
      Point ptCamera(_cameraPos[0], _cameraPos[1], _cameraPos[2]);
      bRet = MoveCameraAxis(tree, pt, oivi, ptCamera, fp);
    }
    break;
  }
  fclose(fp);
  
  if (!bRet)std::cout << "failed...\n";
  
  return bRet ? 0 : -1;
}
