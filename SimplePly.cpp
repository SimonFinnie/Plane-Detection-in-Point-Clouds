#include "SimplePly.h"
#include "rply.h"
#include <iostream>
#include <fstream>

int vertexHandler(p_ply_argument argument) {
  long elemIx, vertIx;
  std::vector<PlyPoint> *points;
  ply_get_argument_element(argument, NULL, &vertIx);
  ply_get_argument_user_data(argument, (void**)(&points), &elemIx);

  switch(elemIx) {
  case 0:
    (*points)[vertIx].location(0) = ply_get_argument_value(argument);
    break;
  case 1:
    (*points)[vertIx].location(1) = ply_get_argument_value(argument);
    break;
  case 2:
    (*points)[vertIx].location(2) = ply_get_argument_value(argument);
    break;
  case 3:
    (*points)[vertIx].colour(0) = ply_get_argument_value(argument);
    break;
  case 4:
    (*points)[vertIx].colour(1) = ply_get_argument_value(argument);
    break;
  case 5:
    (*points)[vertIx].colour(2) = ply_get_argument_value(argument);
    break;
  default:
    std::cerr << "Unrecognised element type " << elemIx << std::endl;
    return 0;
  }
  return 1;
}

bool SimplePly::read(const std::string& filename) {
  p_ply ply = ply_open(filename.c_str(), NULL, 0, NULL);
  if (!ply) {
    std::cerr << "Failed to open PLY file" << std::endl;
    return false;
  }
  if (!ply_read_header(ply)) {
    std::cerr << "Failed to read PLY header" << std::endl;
    return false;
  }
  
  long nVerts = ply_set_read_cb(ply, "vertex", "x", vertexHandler, &points_, 0);
  ply_set_read_cb(ply, "vertex", "y", vertexHandler, &points_, 1);
  ply_set_read_cb(ply, "vertex", "z", vertexHandler, &points_, 2);
  ply_set_read_cb(ply, "vertex", "r", vertexHandler, &points_, 3);
  ply_set_read_cb(ply, "vertex", "g", vertexHandler, &points_, 4);
  ply_set_read_cb(ply, "vertex", "b", vertexHandler, &points_, 5);
  ply_set_read_cb(ply, "vertex", "red", vertexHandler, &points_, 3);
  ply_set_read_cb(ply, "vertex", "green", vertexHandler, &points_, 4);
  ply_set_read_cb(ply, "vertex", "blue", vertexHandler, &points_, 5);
  points_.resize(nVerts);

    ply_read(ply);

  return true;
}

bool SimplePly::write(const std::string& filename) {
  std::ofstream fout(filename);
  if (!fout) {
    std::cerr << "Failed to open PLY file " << filename << " for writing" << std::endl;
    return false;
  }
  
  fout << "ply\n"
       << "format ascii 1.0\n"
       << "element vertex " << size() << "\n"
       << "property float x\n"
       << "property float y\n"
       << "property float z\n"
       << "property uchar red\n"
       << "property uchar green\n"
       << "property uchar blue\n"
       << "end_header\n";

  for (size_t ix = 0; ix < size(); ++ix) {
    fout << points_[ix].location.transpose() << " " << points_[ix].colour.transpose() << "\n";
  }
  fout.close();
  return true;
}
