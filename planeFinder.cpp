#include "SimplePly.h"
#include "rply.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

/**Calculates the distance the point is from the plane
   @param point a single point's position.
   @param planeEq plane Equation.
   @return distance between point and plane.
 */
float distToPlane(Eigen::Vector3f point, Eigen::Vector4f planeEq){
  float distance = planeEq(0)*point(0) + planeEq(1)*point(1) + planeEq(2)*point(2) + planeEq(3);
  distance /= std::sqrt(std::pow(planeEq(0),2) + std::pow(planeEq(1),2) + std::pow(planeEq(2),2));
  distance = std::abs(distance);
  return distance;
}

/**builds a plane out of 3 points
   @param point list of point positions.
   @return the equation of the plane.
 */
Eigen::Vector4f calcPlane(std::vector<Eigen::Vector3f> points){
  Eigen::Vector3f vectAB = points[1] - points[0], vectAC = points[2] - points[0];
  Eigen::Vector3f planeNorm = vectAB.cross(vectAC);
  float planeConst = -points[0].dot(planeNorm);
  Eigen::Vector4f planeEquation(planeNorm(0),planeNorm(1),planeNorm(2),planeConst);
  return planeEquation;
}

/**finds the distance that a point (with the greatest distance from the plane) has
   @param points list of points positions.
   @param planeEq the equation for the plane.
   @return the max distance between any point and the plane.
 */ 
float maxDistance(std::vector<Eigen::Vector3f> points, Eigen::Vector4f planeEq){
  float maxDist = 0, dist = 0;
  for(int i = 0; i < points.size(); i++){
    if((dist = distToPlane(points[i], planeEq)) > maxDist){
      maxDist = dist;
    }
  }
  return maxDist;
}

/**calculates the centroid of the points given.
   @param points list of points positions.
   @return the centroid of the points positions.
 */
Eigen::Vector3f calcCentroid(std::vector<Eigen::Vector3f> points){
  float aveX = 0, aveY = 0, aveZ = 0;
  Eigen::Vector3f centroid;
  for(int i = 0; i < points.size(); i++){
    aveX += points[i](0);
    aveY += points[i](1);
    aveZ += points[i](2);
  }
  //averaging the values.
  aveX/=points.size();
  aveY/=points.size();
  aveZ/=points.size();
  centroid(0) = aveX;
  centroid(1) = aveY;
  centroid(2) = aveZ;
  return centroid;
}

/** Preforms orthogonal least squares on the points given to find the best plane. Then the
    method projects the points onto the plane.
    @param points list of points location.
    @param plyPoints list of points in ply format.
*/
void leastSquares(std::vector<Eigen::Vector3f> points, std::vector<PlyPoint *> &plyPoints){
  Eigen::MatrixXf pointsMat(points.size(),3), pointsV;
  Eigen::Vector3f centroid = calcCentroid(points), planeNorm, adjustedPoint;
  Eigen::Vector4f planeEq;
  float planeNormNorm = 0, dist = 0;
  //builds matrix to use SVD on.
  for(int i = 0; i < points.size(); i++){
    for(int j = 0; j < 3; j++){
      pointsMat(i,j) = points[i](j) - centroid(j);
    }
  }
  //preforms SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(pointsMat, Eigen::ComputeFullV);
  pointsV = svd.matrixV();
  //builds plane based off of the SVD eigen vector and the centroid.
  for(int i = 0; i < 3; i++){
    planeEq(i) = pointsV(i,0);
    planeEq(3) -= planeEq(i)*centroid(i);
  }
  //finds normal of the plane normal.
  for(int i = 0; i < 3; i++){
    planeNormNorm += std::pow(planeEq(i),2);
  }
  planeNormNorm = std::sqrt(planeNormNorm);
  //builds a normalised plane normal vector.
  for(int i = 0; i < 3; i++){
    planeNorm(i) = planeEq(i)/planeNormNorm;
  }
  //projects points positions onto the calcuated Plane.
  for(int i = 0; i < plyPoints.size(); i++){
    dist = distToPlane(points[i],planeEq);
    
    adjustedPoint(0)= points[i](0) - (dist*planeNorm(0));
    adjustedPoint(1)= points[i](1) - (dist*planeNorm(1));
    adjustedPoint(2)= points[i](2) - (dist*planeNorm(2));
    if(dist < distToPlane(adjustedPoint,planeEq)){
      adjustedPoint(0)= points[i](0) + (2*dist*planeNorm(0));
      adjustedPoint(1)= points[i](1) + (2*dist*planeNorm(1));
      adjustedPoint(2)= points[i](2) + (2*dist*planeNorm(2));
    }
    
    (*plyPoints[i]).location(0) = adjustedPoint(0);
    (*plyPoints[i]).location(1) = adjustedPoint(1);
    (*plyPoints[i]).location(2) = adjustedPoint(2);
  }
}






/** Based on a percentage of points in the biggest plane, it calculates the threshold for the plane.
 * @param points the list of points positions.
 * @param planeEq the equation for the plane.
 * @param probThreshold the percentage of points required to determine threshold.
 * @return the calculated threshold.
 */
float calcThreshold(std::vector<Eigen::Vector3f> points, Eigen::Vector4f planeEq, float probThreshold){
  std::vector<float> buckets;
  float maxDist = maxDistance(points, planeEq);
  int numJumps = 10000, pointCount = 0, placeHold = 0, reqPoints = std::ceil(points.size()*probThreshold);
  float jumpSize = maxDist/(numJumps-1), dist, threshold = 0;
    
  for(int i = 0; i < numJumps; i++){
    buckets.push_back(0);
  }
  //Counts the number of Points in each region bucket.
  for(int i = 0; i < points.size(); i++){
    dist = distToPlane(points[i],planeEq);
    buckets[std::floor(dist/jumpSize)]++;
  }
  //sums point count in buckets until it goes over the users estimated plane count.
  while(pointCount < reqPoints){
    pointCount += buckets[placeHold];
    placeHold++;
  }
  //sets that point as threshold.
  threshold = (placeHold+1)*jumpSize;
    
  return threshold;

}




/**Determines the number of trials needed to have the accuracy that is requested.
 * @param inlierProb the probability of a point being an inlier.
 * @param acceptedFailProb the accepted rate of failure.
 *@return the number of trials required.
 */
float numChecks(float inlierProb, float acceptedFailProb){
  float numChecks = 0;
  if(inlierProb < 1){
    numChecks = ((std::log(acceptedFailProb))/(std::log(1 - (std::pow(inlierProb,3)))));
  }
  
  return numChecks;
}


/**Determines if the plane is a null plane.
   @param plane the plane equation.x
   @return a statement of if the plane is a null plane.
 */
bool planeNull(Eigen::Vector4f plane){
  return (abs(plane(0)) == 0 && plane(1) == 0 && plane(2) == 0 && plane(3) == 0);
}







/**Proforms analysis on the inputed point cloud, to find planes in the point cloud, colouring
   them accordingly. The program requires the user to input some information about the data,
   such as a points limit to when to stop looking for planes, an estimate of the percentage
   of points in the largest plane, and the probability of success.
*/
int main (int argc, char *argv[]) {
  int pointIn, count, bestCount, point1In, point2In, point3In, reqPlaneCalcs = 0;
  float posThresh = 0, threshProb = 0.4, threshold = 0, inlierProb = 0.001;
  Eigen::Vector4f bestPlane(0,0,0,0), curPlane;
  Eigen::Vector3f holdPoint;
  Eigen::Vector3i baseColour;
  std::vector<Eigen::Vector3f> points, pointsList, planePoints;
  int trialCount = 0,planeCount = 0;
  bool stillPlanes = true, threshFound = false;
  // Check the commandline arguments.
  if (argc != 6) {
    std::cout << "Usage: planeFinder <input file> <output file> <min num of points in plane> <percentage of points in largest plane> <accepted failure rate>" << std::endl;
    return -1;
  }
  int planeSizeThreshold = atoi(argv[3]);
  float pointPercentBiggestPlane = atof(argv[4]);
  float acceptedFailProb = atof(argv[5]);
  std::vector<PlyPoint *> pointsList2, planePoints2;
  
  reqPlaneCalcs = numChecks(pointPercentBiggestPlane, acceptedFailProb);
  threshold = std::numeric_limits<float>::max();
  std::cout << "Searching for planes until the next best plane size is less than " << planeSizeThreshold << std::endl;
  std::cout << "The estimated percentage of points in the biggest plane is " << pointPercentBiggestPlane << std::endl;
  std::cout << "Applying RANSAC with an acceptedFailRate of " << acceptedFailProb << std::endl;  

  // Storage for the point cloud.
  SimplePly ply;
  // Read in the data from a PLY file
  std::cout << "Reading PLY data from " << argv[1] << std::endl;
  if (!ply.read(argv[1])) {
    std::cout << "Could not read PLY data from file " << argv[1] << std::endl;
    return -1;
  }

  // Stores copy of points read in.
  for(int i = 0; i < ply.size(); ++i){
    pointsList2.push_back(&ply[i]);
  }
  
  
  std::cout << "Read " << ply.size() << " points" << std::endl;
  
  // Recolour points - here we are just doing colour based on index
  std::cout << "Recolouring points" << std::endl;
  
  //Sets colours
  std::vector<Eigen::Vector3i> colours;
  colours.push_back(Eigen::Vector3i(255,0,0));
  colours.push_back(Eigen::Vector3i(0,255,0));
  colours.push_back(Eigen::Vector3i(0,0,255));
  colours.push_back(Eigen::Vector3i(255,255,0));
  colours.push_back(Eigen::Vector3i(0,255,255));
  colours.push_back(Eigen::Vector3i(255,0,255));
  colours.push_back(Eigen::Vector3i(255,255,255));
  colours.push_back(Eigen::Vector3i(0,0,0));
  
  holdPoint = Eigen::Vector3f(ply[0].location(0),ply[0].location(1),ply[0].location(2));
  
  
  
  //builds a vector containing vectors of the points positions.
  for(int i = 0; i < ply.size(); i++){
    holdPoint = Eigen::Vector3f(ply[i].location(0),ply[i].location(1),ply[i].location(2));
    pointsList.push_back(holdPoint);
  }

  
  

  //starts looking for planes.
  while(stillPlanes && pointsList.size() > 2){
    //zeros bestPlane.
    for(int j = 0; j < 4; j++){
      bestPlane(j) = 0;
    }
    
    //increase plane Count.

    planeCount++;
    //Resets variables.
    bestCount = 0;
    inlierProb = 0.001;
    trialCount = 0;
    
    //finds the threshold based off of users estimate.
    if(!threshFound){
      for(int z = 0; z < 110; z++){
	point1In = std::rand()%pointsList.size();
	point2In = std::rand()%pointsList.size();
	point3In = std::rand()%pointsList.size();
	points.clear();
	points.push_back(pointsList[point1In]);
	points.push_back(pointsList[point2In]);
	points.push_back(pointsList[point3In]);
	curPlane = calcPlane(points);
	if(!planeNull(curPlane)){
	  posThresh = calcThreshold(pointsList, curPlane, pointPercentBiggestPlane);
	  if(posThresh < threshold){
	    threshold = posThresh;
	    bestPlane = curPlane;
	  }
	}
      }
      //makes sure only one threshold is found.
      threshFound = true;
    }



    //starts looking for planes with the calculated threshold.
    while(trialCount <  numChecks(inlierProb, acceptedFailProb)){
      trialCount++;
      std::cout << numChecks(inlierProb, acceptedFailProb) << std::endl;
      //finds three random points and builds a plane.
      point1In = std::rand()%pointsList.size();
      point2In = std::rand()%pointsList.size();
      point3In = std::rand()%pointsList.size();
      points.clear();
      points.push_back(pointsList[point1In]);
      points.push_back(pointsList[point2In]);
      points.push_back(pointsList[point3In]);
      curPlane = calcPlane(points);
      count = 0;
      //makes sure it isn't a null plane.
      if(!planeNull(curPlane)){
	for(int k = 0; k < pointsList.size(); k++){
	  //counts number of points within threshold.
	  if(distToPlane(pointsList[k], curPlane) <= threshold){
	    count++;
	  }
	}
	//checks if this is best plane then updates if it is.
	if(count > bestCount){
	  bestCount = count;
	  bestPlane = curPlane;
	  inlierProb = ((float)bestCount)/pointsList.size();
	  
	}
      }
    }
    //Cuts out if the plane is too small.
    if(bestCount < planeSizeThreshold){
      stillPlanes = false;
    }
    //Makes sure the unlikely chance of a null best plane occuring doesn't get through.
    else if(!planeNull(bestPlane)){
      //recolours and removes points.
      for(int j = pointsList2.size() - 1; j > -1; j--){
	holdPoint(0) = (*pointsList2[j]).location(0);
	holdPoint(1) =  (*pointsList2[j]).location(1);
	holdPoint(2) =  (*pointsList2[j]).location(2);
	
	if(distToPlane(holdPoint, bestPlane) <= threshold){
	  
	  (*pointsList2[j]).colour = colours[planeCount%colours.size()];
	  //planePoints.push_back(pointsList[j]);
	  pointsList[j] = pointsList[pointsList.size() - 1];
	  pointsList.pop_back();
	  //planePoints2.push_back(pointsList2[j]);
	  pointsList2[j] = pointsList2[pointsList2.size() - 1];
	  pointsList2.pop_back();
	
	}
      }
      //would do least squares if it worked properly. No time to work out the error.
      //leastSquares(planePoints, planePoints2);
      //planePoints.clear();
      //planePoints2.clear();
    }
  }
  
 
  // Write the resulting (re-coloured) point cloud to a PLY file.
  std::cout << "Writing PLY data to " << argv[2] << std::endl;
  if (!ply.write(argv[2])) {
    std::cout << "Could not write PLY data to file " << argv[2] << std::endl;
    return -2;
  }
  return 0;
}









//Failed methods for calculating threshold.
/*
float calcThresh(std::vector<Eigen::Vector3f> points, Eigen::Vector4f planeEq){
  std::vector<int> buckets;
  int numJumps = 1000, max, biggestDif = points.size();
  float maxDist = maxDistance(points, planeEq), jumpSize = maxDist/numJumps, dist = 0, aveVal = 0, totVal = 0, threshold = 0;
  for(int i = 0; i < numJumps; i++){
    buckets.push_back(0);
  }
  for(int i = 0; i < points.size(); i++){
    dist = distToPlane(points[i],planeEq);
    buckets[std::floor(dist/jumpSize)]++;
  }
  totVal = std::abs(buckets[0] - buckets[1]);
  aveVal = std::abs(buckets[0] - buckets[1]);
  for(int i = 2; i < buckets.size(); i++){
    if(std::abs(buckets[i-1] - buckets[i]) > 6000){
      threshold = (i)*jumpSize;
      i = buckets.size();
    }
    else{
      totVal += std::abs(buckets[i-1] - buckets[i]);
      aveVal = totVal/(i);
      std::cout << "ah " <<  aveVal << std::endl;
    }
  }
  return threshold;
}


float calcThreshold(std::vector<Eigen::Vector3f> points, Eigen::Vector4f planeEq){
  std::vector<int> buckets;
  float maxDist = maxDistance(points, planeEq), reqPoints = points.size()*probThreshold;
  int numJumps = 10000, pointCount = 0, placeHold = 0;
  float jumpSize = maxDist/(numJumps-1), dist, maxCount = 0, threshold = 0;
   /*

  for(int i = 0; i < reqPoints; i++){
    //std::cout << "hello2 " << std::endl;
    distances.push_back(distToPlane(points[i],planeEq));
    //
    
    sort(distances);
    std::cout << i << " " << reqPoints << std::endl;
    //std::cout << "hello3 " << std::endl;
  }
  std::cout << "hello4 " << std::endl;
  for(int i = reqPoints; i < points.size(); i++){
    dist = distToPlane(points[i],planeEq);
    if(dist < distances[distances.size()-1]){
      distances[distances.size()-1] = dist;
      sort(distances);
    }
  }
  for(int i = 0; i < distances.size(); i++){
    std::cout << distances[i] << std::endl;
  }
  return distances[distances.size()-1];
  /*
  //std::cout << "hello2 " << std::endl;
  for(int i = 0; i < points.size(); i++){
    dist = distToPlane(points[i],planeEq);
    buckets[std::floor(dist/jumpSize)]++;
  }
  while(pointCount < reqPoints){
    pointCount += buckets[placeHold];
    placeHold++;
  }
  threshold = (placeHold+1)*jumpSize;
  

  std::cout << "pg " << std::endl;
  for(int i = 0; i < buckets.size(); i++){
    if(buckets[i] > 0){
      std::cout << "p " << i << " " << buckets[i] << std::endl;
    }
  }
  std::cout << "pg " << threshold << std::endl;
  return threshold;
}


void sort(std::vector<float> &points){
  //std::cout << "hil " << std::endl;
  float hold = 0;
  int end = points.size()-1;
  if(points[end-1] > points[end]){
    for(int i = end; i > 0 && points[i-1] > points[i]; i--){
      hold = points[i-1];
      points[i-1] = points[i];
      points[i] = hold;
    }
  }
}
*/
