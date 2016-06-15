#include <egraphs/egraph_decomposer.h>

using namespace std;

vector<EGraph*> EGraphDecomposer::decompose(const EGraph& eg, int num_egs, const EGraphable<vector<double> >& env){
  ROS_INFO("decomposing...");

  //vector<vector<vector<double> > > filtered_paths;
  //vector<vector<int> > filtered_path_costs;
  //vector<sbpl::PathSimilarityMeasurer::Trajectory> xyz_paths;
  //convertEGraphPathsToXYZPaths(eg, env, filtered_paths, filtered_path_costs, xyz_paths);
  //ROS_INFO("we have %d xyz_paths",xyz_paths.size());

  //egraph, paths for that egraph, waypoints in that path, coord of that waypoint
  vector<vector<vector<vector<double> > > > paths;
  //egraph, path, costs
  vector<vector<vector<int> > > path_costs;
  paths.resize(num_egs);
  path_costs.resize(num_egs);
    
  /*
  if(num_egs <= 30){
    vector<vector<double> > distMatrix;
    buildDistanceMatrix(xyz_paths,distMatrix);
    ROS_INFO("dist matrix size %d",distMatrix.size());

    bool cross_validate = false;
    if(cross_validate){
      vector<double> var;
      for(int i=num_egs; i<sqrt(xyz_paths.size()/2.0); i++){
        vector<vector<int> > temp_clusters;
        KMedoids(distMatrix, i, temp_clusters);
        double variance = computeClusterVariance(distMatrix, temp_clusters);
        var.push_back(variance);
      }
      vector<double> dvar;
      for(unsigned int i=0; i<var.size(); i++)
        printf("%f ",var[i]);
      printf("\n");
      for(unsigned int i=1; i<var.size(); i++){
        dvar.push_back(var[i]-var[i-1]);
        printf("%f ",dvar.back());
      }
      printf("\n");
      exit(1);
    }

    int k = 16;
    bool use_rule_of_thumb = false;
    if(use_rule_of_thumb)
      k = sqrt(xyz_paths.size()/2.0);

    vector<vector<int> > cluster_indices;
    KMedoids(distMatrix, k, cluster_indices);

    bool break_up_clusters = false;

    if(break_up_clusters){
      //method 1: give each egraph a part of each cluster
      int cnt = 0;
      for(unsigned int i=0; i<cluster_indices.size(); i++){
        for(unsigned int j=0; j<cluster_indices[i].size(); j++){
          int cluster_idx = cluster_indices[i][j];
          //ROS_INFO("(%dth path from cluster %d) give path %d to egraph %d",j,i,cluster_idx,cnt);
          paths[cnt].push_back(filtered_paths[cluster_idx]);
          path_costs[cnt].push_back(filtered_path_costs[cluster_idx]);
          cnt = (cnt + 1) % num_egs;
        }
      }
    }
    else{
      //method 2: each cluster goes to only 1 egraph
      int cnt = 0;
      for(unsigned int i=0; i<cluster_indices.size(); i++){
        for(unsigned int j=0; j<cluster_indices[i].size(); j++){
          int cluster_idx = cluster_indices[i][j];
          //ROS_INFO("(%dth path from cluster %d) give path %d to egraph %d",j,i,cluster_idx,cnt);
          paths[cnt].push_back(filtered_paths[cluster_idx]);
          path_costs[cnt].push_back(filtered_path_costs[cluster_idx]);
        }
        cnt = (cnt + 1) % num_egs;
      }
    }
  }
  else{
    ROS_INFO("too many heuristics, not using clustering...");
    int cnt = 0;
    for(unsigned int i=0; i<filtered_paths.size(); i++){
      paths[cnt].push_back(filtered_paths[i]);
      path_costs[cnt].push_back(filtered_path_costs[i]);
      cnt = (cnt + 1) % num_egs;
    }
  }
  */
  int cnt = 0;
  for(unsigned int i=0; i<eg.paths.size(); i++){
    paths[cnt].push_back(eg.paths[i]);
    path_costs[cnt].push_back(eg.path_costs[i]);
    cnt = (cnt + 1) % num_egs;
  }

  vector<EGraph*> egs(num_egs,NULL);
  for(int i=0; i<num_egs; i++){
    egs[i] = new EGraph(eg.eg_disc_, eg.num_dims_, eg.num_constants_);
    egs[i]->addPaths(paths[i], path_costs[i]);
  }

  return egs;
}

void EGraphDecomposer::convertEGraphPathsToXYZPaths(const EGraph& eg, const EGraphable<vector<int> >& env, 
                                                    vector<vector<vector<double> > >& filtered_paths,
                                                    vector<vector<int> >& filtered_path_costs,
                                                    vector<sbpl::PathSimilarityMeasurer::Trajectory>& xyz_paths){
  for(unsigned int i=0; i<eg.paths.size(); i++){
    sbpl::PathSimilarityMeasurer::Trajectory t;
    for(unsigned int j=0; j<eg.paths[i].size(); j++){
      vector<int> h_coord;
      env.projectToHeuristicSpace(eg.paths[i][j], h_coord);
      geometry_msgs::Point p;
      p.x = h_coord[0];
      p.y = h_coord[1];
      p.z = h_coord[2];
      if(j>0 && p.x==t.back().x && p.y==t.back().y && p.z==t.back().z)
        continue;
      t.push_back(p);
    }
    if(t.size()<=1)
      continue;
    xyz_paths.push_back(t);
    filtered_paths.push_back(eg.paths[i]);
    filtered_path_costs.push_back(eg.path_costs[i]);
  }
}

void EGraphDecomposer::buildDistanceMatrix(const vector<sbpl::PathSimilarityMeasurer::Trajectory>& paths, vector<vector<double> >& distMatrix){
  distMatrix.resize(paths.size());
  for(unsigned int i=0; i<paths.size(); i++){
    distMatrix[i].resize(paths.size());
    distMatrix[i][i] = 0;
  }
  for(unsigned int i=0; i<paths.size(); i++){
    for(unsigned int j=i+1; j<paths.size(); j++){
      vector<const sbpl::PathSimilarityMeasurer::Trajectory*> trajectories;
      const sbpl::PathSimilarityMeasurer::Trajectory* traj1 = &paths[i];
      const sbpl::PathSimilarityMeasurer::Trajectory* traj2 = &paths[j];
      trajectories.push_back(traj1);
      trajectories.push_back(traj2);
      for(unsigned int k=0; k<traj1->size(); k++){
        if(std::isnan(traj1->at(k).x) ||
           std::isnan(traj1->at(k).y) ||
           std::isnan(traj1->at(k).z)){
          ROS_INFO("poopy");
          std::cin.get();
        }
      }
      for(unsigned int k=0; k<traj2->size(); k++){
        if(std::isnan(traj2->at(k).x) ||
           std::isnan(traj2->at(k).y) ||
           std::isnan(traj2->at(k).z)){
          ROS_INFO("poopy");
          std::cin.get();
        }
      }
      double d = sbpl::PathSimilarityMeasurer::measureDTW(trajectories, max(paths[i].size(), paths[j].size()));
      bool sadness = std::isnan(d);
      if(sadness){
        ROS_ERROR("sadness...");
        std::cin.get();

        ROS_INFO("traj1");
        for(unsigned int k=0; k<traj1->size(); k++){
            ROS_INFO("%f %f %f",
                traj1->at(k).x,
                traj1->at(k).y,
                traj1->at(k).z);
        }
        ROS_INFO("traj2");
        for(unsigned int k=0; k<traj2->size(); k++){
            ROS_INFO("%f %f %f",
                traj2->at(k).x,
                traj2->at(k).y,
                traj2->at(k).z);
        }
        std::cin.get();

      }
      distMatrix[i][j] = d;
      distMatrix[j][i] = d;
    }
  }
}

void EGraphDecomposer::KMedoids(const vector<vector<double> >& dists, int k, vector<vector<int> >& final_clusters){
  //randomly choose medoids
  vector<int> rand_init(dists.size(),0);
  for(unsigned int i=0; i<rand_init.size(); i++)
    rand_init[i] = i;
  for(int i=0; i<k; i++){
    int r = (rand() % (rand_init.size() - i)) + i;
    swap(rand_init[i], rand_init[r]);
  }

  //initialize clusters
  vector<pair<int, vector<int> > > best_clusters;
  vector<pair<int, vector<int> > > clusters;
  clusters.resize(k);
  best_clusters.resize(k);
  for(int i=0; i<k; i++)
    clusters[i].first = rand_init[i];
  double best_cost = computeClustersFromMedoids(clusters, dists, best_clusters);
  ROS_INFO("initial best_cost %f",best_cost);
  //at this point clusters has the proper centers but membership is not right (it is correct in best_clusters)
  
  //repeat until convergence
  bool medoids_changed = true;
  while(medoids_changed){
    clusters = best_clusters;//update clusters to the best_clusters found in the previous iteration
    ROS_INFO("start k-medoid iteration");
    medoids_changed = false;
    for(unsigned int i=0; i<clusters.size(); i++){//for each cluster
      for(unsigned int j=0; j<clusters[i].second.size(); j++){//try swapping each member for the center
        swap(clusters[i].first, clusters[i].second[j]);
        vector<pair<int, vector<int> > > temp_clusters;
        double cost = computeClustersFromMedoids(clusters, dists, temp_clusters);
        if(cost < best_cost){
          best_cost = cost;
          best_clusters = temp_clusters;
          medoids_changed = true;
          ROS_INFO("best_cost updated to %f",best_cost);
        }
        swap(clusters[i].first, clusters[i].second[j]);
      }
    }
  }
  ROS_INFO("converged");

  //package returned clusters
  for(unsigned int i=0; i<best_clusters.size(); i++){
    final_clusters.push_back(best_clusters[i].second);
    final_clusters.back().push_back(best_clusters[i].first);
  }

  set<int> s;
  for(unsigned int i=0; i<final_clusters.size(); i++){
    for(unsigned int j=0; j<final_clusters[i].size(); j++){
      int v = final_clusters[i][j];
      assert(v >= 0);
      assert(v < int(dists.size()));
      assert(v < 1500);
      assert(s.find(v) == s.end());
      s.insert(v);
    }
  }

}

double EGraphDecomposer::computeClustersFromMedoids(const vector<pair<int, vector<int> > >& medoids, const vector<vector<double> >& dists, vector<pair<int, vector<int> > >& clusters){
  clusters.clear();
  clusters.resize(medoids.size());
  for(unsigned int i=0; i<medoids.size(); i++)
    clusters[i].first = medoids[i].first;

  double total_cost = 0;
  unsigned int skips = 0;
  for(unsigned int i=0; i<dists.size(); i++){
    double best_cost = numeric_limits<double>::max();
    int best_medoid = -1;
    bool isMedoid = false;
    for(unsigned int j=0; j<medoids.size(); j++){
      unsigned int medoid = medoids[j].first;
      if(i==medoid){
        //ROS_INFO("%d is a medoid",i);
        isMedoid = true;
        break;
      }
      double cost = dists[i][medoid];
      //ROS_INFO("dist[%d][%d] = %f",i,medoid,cost);
      if(cost < best_cost){
        //ROS_INFO("%d found a closer medoid %d",i,j);
        best_cost = cost;
        best_medoid = j;
      }
    }
    if(!isMedoid){
      //ROS_INFO("best_medoid=%d size_clusters=%d i=%d",best_medoid,clusters.size(),i);
      clusters[best_medoid].second.push_back(i);
      total_cost += best_cost;
    }
    else
      skips++;
  }
  assert(skips==medoids.size());

  return total_cost;
}

double EGraphDecomposer::computeClusterVariance(const vector<vector<double> >& dists, const vector<vector<int> >& clusters){
  double w = 0;
  for(unsigned int k=0; k<clusters.size(); k++){
    double d = 0;
    for(unsigned int i=0; i<clusters[k].size(); i++){
      for(unsigned int j=0; j<clusters[k].size(); j++){
        int idx1 = clusters[k][i];
        int idx2 = clusters[k][j];
        d += (dists[idx1][idx2] * dists[idx1][idx2])/100000.0;
      }
    }
    assert(d>=0);
    w += d/(2*clusters[k].size());
    assert(w>=0);
  }
  return w;
}

