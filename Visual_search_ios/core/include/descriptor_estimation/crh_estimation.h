#ifndef CRH_ESTIMATION
#define CRH_ESTIMATION

#include <pcl/features/feature.h>
#include <pcl/common/fft/kiss_fftr.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/histogram_visualizer.h>//DBG
#include <pcl/visualization/pcl_visualizer.h>//DBG

namespace pcl
{
	/** \brief CRHEstimation estimates the Camera Roll Histogram (CRH) descriptor for a given
	* point cloud dataset containing XYZ data and normals, as presented in:
	*   - CAD-Model Recognition and 6 DOF Pose Estimation
	*     A. Aldoma, N. Blodow, D. Gossow, S. Gedikli, R.B. Rusu, M. Vincze and G. Bradski
	*     ICCV 2011, 3D Representation and Recognition (3dRR11) workshop
	*     Barcelona, Spain, (2011)
	*
	* The suggested PointOutT is pcl::Histogram<90>. //dc (real) + 44 complex numbers (real, imaginary) + nyquist (real)
	*
	* \author Aitor Aldoma
	* \ingroup features
	*/
	template<typename PointInT, typename PointNT, typename PointOutT = pcl::Histogram<90> >
	class CRHEstimationDBG : public FeatureFromNormals<PointInT, PointNT, PointOutT>
	{
	public:
		typedef boost::shared_ptr<CRHEstimation<PointInT, PointNT, PointOutT> > Ptr;
		typedef boost::shared_ptr<const CRHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;

		using Feature<PointInT, PointOutT>::feature_name_;
		using Feature<PointInT, PointOutT>::getClassName;
		using Feature<PointInT, PointOutT>::indices_;
		using Feature<PointInT, PointOutT>::k_;
		using Feature<PointInT, PointOutT>::search_radius_;
		using Feature<PointInT, PointOutT>::surface_;
		using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

		typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

		/** \brief Constructor. */
		CRHEstimationDBG () :
			vpx_ (0), vpy_ (0), vpz_ (0), nbins_ (90)
		{
			k_ = 1;
			feature_name_ = "CRHEstimation";
		}
		;

		/** \brief Set the viewpoint.
		* \param[in] vpx the X coordinate of the viewpoint
		* \param[in] vpy the Y coordinate of the viewpoint
		* \param[in] vpz the Z coordinate of the viewpoint
		*/
		inline void
		setViewPoint (float vpx, float vpy, float vpz)
		{
			vpx_ = vpx;
			vpy_ = vpy;
			vpz_ = vpz;
		}

		/** \brief Get the viewpoint.
		* \param[out] vpx the X coordinate of the viewpoint
		* \param[out] vpy the Y coordinate of the viewpoint
		* \param[out] vpz the Z coordinate of the viewpoint
		*/
		inline void
		getViewPoint (float &vpx, float &vpy, float &vpz)
		{
			vpx = vpx_;
			vpy = vpy_;
			vpz = vpz_;
		}

		inline void
		setCentroid (Eigen::Vector4f & centroid)
		{
			centroid_ = centroid;
		}

	private:
		/** \brief Values describing the viewpoint ("pinhole" camera model assumed).
		* By default, the viewpoint is set to 0,0,0.
		*/
		float vpx_, vpy_, vpz_;

		/** \brief Number of bins, this should match the Output type */
		int nbins_;

		/** \brief Centroid to be used */
		Eigen::Vector4f centroid_;

		/** \brief Estimate the CRH histogram at
		* a set of points given by <setInputCloud (), setIndices ()> using the surface in
		* setSearchSurface ()
		*
		* \param[out] output the resultant point cloud with a CRH histogram
		*/
		void
		computeFeature (PointCloudOut &output);
	};
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointNT, typename PointOutT>
void
pcl::CRHEstimationDBG<PointInT, PointNT, PointOutT>::computeFeature (PointCloudOut &output)
{
    // Check if input was set
    if (!normals_)
    {
        PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.points.clear ();
        return;
    }

    if (normals_->points.size () != surface_->points.size ())
    {
        PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
        output.width = output.height = 0;
        output.points.clear ();
        return;
    }

    Eigen::Vector3f plane_normal;
    plane_normal[0] = -centroid_[0];
    plane_normal[1] = -centroid_[1];
    plane_normal[2] = -centroid_[2];
    Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
    plane_normal.normalize ();
    Eigen::Vector3f axis = plane_normal.cross (z_vector);
    double rotation = -asin (axis.norm ());
    axis.normalize ();

    int nbins = nbins_;
    int bin_angle = 360 / nbins;

    Eigen::Affine3f transformPC (Eigen::AngleAxisf (static_cast<float> (rotation), axis));

    pcl::PointCloud<pcl::PointNormal> grid;
    grid.points.resize (indices_->size ());

    for (size_t i = 0; i < indices_->size (); i++)
    {
        grid.points[i].getVector4fMap () = surface_->points[(*indices_)[i]].getVector4fMap ();
        grid.points[i].getNormalVector4fMap () = normals_->points[(*indices_)[i]].getNormalVector4fMap ();
    }

    pcl::transformPointCloudWithNormals (grid, grid, transformPC);
	////////DBG/////////
	pcl::visualization::PCLVisualizer cloud_visualizer("Transformed PC");
	cloud_visualizer.addPointCloud<pcl::PointNormal>(grid.makeShared(), pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(grid.makeShared(), 0.0, 255.0, 0.0), "transformed cloud");
	cloud_visualizer.addPointCloudNormals<pcl::PointNormal>(grid.makeShared(), 5, 0.05, "transformed cloud normals");
	cloud_visualizer.spin();
	////////////////////

    //fill spatial data vector
    kiss_fft_scalar * spatial_data = new kiss_fft_scalar[nbins];
	for(unsigned int i = 0; i < nbins; ++i)
		spatial_data[i] = 0.0;
		
    float sum_w = 0, w = 0;
    int bin = 0;
    for (size_t i = 0; i < grid.points.size (); ++i)
    {
        bin = static_cast<int> ((((atan2 (grid.points[i].normal_y, grid.points[i].normal_x) + M_PI) * 180.0 / M_PI) / bin_angle)) % nbins;
        w = sqrtf (grid.points[i].normal_y * grid.points[i].normal_y + grid.points[i].normal_x * grid.points[i].normal_x);
        sum_w += w;
        spatial_data[bin] += w;
    }

    for (int i = 0; i < nbins; ++i)
        spatial_data[i] /= sum_w;

	////////////DBG////////////
	pcl::PointCloud<pcl::Histogram<90> > crh_histogram;
	crh_histogram.points.resize(1);
	crh_histogram.width = crh_histogram.height = 1;
	for(unsigned int i = 0; i < nbins; ++i)
		crh_histogram.points[0].histogram[i] = spatial_data[i];
	
	pcl::visualization::PCLHistogramVisualizer crh_visualizer;
	crh_visualizer.addFeatureHistogram<pcl::Histogram<90> >(crh_histogram, 90, "CRH feature");
	crh_visualizer.spin();
	///////////////////////////
	
    kiss_fft_cpx * freq_data = new kiss_fft_cpx[nbins / 2 + 1];
    kiss_fftr_cfg mycfg = kiss_fftr_alloc (nbins, 0, NULL, NULL);
    kiss_fftr (mycfg, spatial_data, freq_data);

    output.points.resize (1);
    output.width = output.height = 1;

    output.points[0].histogram[0] = freq_data[0].r / freq_data[0].r; //dc
    int k = 1;
    for (int i = 1; i < (nbins / 2); i++, k += 2)
    {
        output.points[0].histogram[k] = freq_data[i].r / freq_data[0].r;
        output.points[0].histogram[k + 1] = freq_data[i].i / freq_data[0].r;
    }

    output.points[0].histogram[nbins - 1] = freq_data[nbins / 2].r / freq_data[0].r; //nyquist

    delete[] spatial_data;
    delete[] freq_data;

}

#endif