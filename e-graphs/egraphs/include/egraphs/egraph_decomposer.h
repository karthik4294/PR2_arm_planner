#include <sbpl_geometry_utils/PathSimilarityMeasurer.h>
#include <egraphs/egraph.h>
#include <egraphs/egraphable.h>
#include <vector>

class EGraphDecomposer{

  public:

    static std::vector<EGraph*> decompose(const EGraph& eg, int num_egs, const EGraphable<std::vector<double> >& env);
    //static std::vector<EGraph*> decompose(const EGraph& eg, int num_egs, const EGraphable<std::vector<int> >& env);

  private:

    //static void convertEGraphPathsToXYZPaths(const EGraph& eg, const EGraphable<std::vector<int> >& env, 
        //std::vector<sbpl::PathSimilarityMeasurer::Trajectory>& paths);
    static void convertEGraphPathsToXYZPaths(const EGraph& eg, const EGraphable<std::vector<int> >& env, 
                                                    std::vector<std::vector<std::vector<double> > >& filtered_paths,
                                                    std::vector<std::vector<int> >& filtered_path_costs,
                                                    std::vector<sbpl::PathSimilarityMeasurer::Trajectory>& xyz_paths);

    static void buildDistanceMatrix(const std::vector<sbpl::PathSimilarityMeasurer::Trajectory>& paths, 
        std::vector<std::vector<double> >& distMatrix);

    static void KMedoids(const std::vector<std::vector<double> >& dists, int k, 
        std::vector<std::vector<int> >& final_clusters);

    static double computeClustersFromMedoids(const std::vector<std::pair<int, std::vector<int> > >& medoids, 
        const std::vector<std::vector<double> >& dists, std::vector<std::pair<int, std::vector<int> > >& clusters);

    static double computeClusterVariance(const std::vector<std::vector<double> >& dists, const std::vector<std::vector<int> >& clusters);
};
