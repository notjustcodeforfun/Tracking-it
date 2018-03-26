#include "../include/Tracked_object.h"


namespace Conveyor_object_tracking {

Tracked_object::Tracked_object ( const Observed_object& observation , Eigen::Vector3f objPosition)
	: _first_time( observation.time ),
	  _last_time( observation.time ),
	  _radii(observation.sliceRadii),
	  _wallThickness(observation.wallThickness),
	  _handleHull(observation.handleHull),
	  _height( observation.height),
	  _position( observation.position ),
	  _velocity( 0.0, 0.0, 0.0 ),
	  _position_cov( observation.covariance ),
	  _velocity_cov( 1.0 ), // scalar noise factors
	  unobserved( 0 ), age( 0 ),
	  _last_object_position( objPosition ),
	  degreesReconstructed( 0.0 )
{
}//Tracked_object::Tracked_object

void Tracked_object::predict_position ( const ros::Time time, Eigen::Vector3f* prediction, double* prediction_cov ) const {
	double interval = time.toSec() - _last_time.toSec();
	*prediction = _position + _velocity * interval;
	*prediction_cov = _position_cov + interval / 60.0;
}//Tracked_object::predict_position

void Tracked_object::predict_velocity ( const ros::Time time, Eigen::Vector3f* prediction, double* prediction_cov ) const {
	double interval = time.toSec() - _last_time.toSec();
	*prediction = _velocity;
	*prediction_cov = _velocity_cov + interval / 60.0;
}//Tracked_object::predict_velocity

void Tracked_object::update_position ( const Observed_object& observation ) {
	double interval = observation.time.toSec() - _last_time.toSec();

	// old predictions are more inaccurate
	double process_cov = interval / 60.0;
	
	// update object height
	if (observation.height > _height) { _height = observation.height; }
	
	
	// update covs
        _position_cov += process_cov;
        _position_cov /= (_position_cov + observation.covariance);
        _velocity_cov += process_cov;
        _velocity_cov /= (_velocity_cov + observation.covariance);

	Eigen::Vector3f next_predicted_position = _position + _velocity * interval;

	Eigen::Vector3f updated_position = next_predicted_position * ( 1.0 - _position_cov ) + observation.position * _position_cov;

	Eigen::Vector3f updated_velocity = ( updated_position - _position ) / interval;

	_velocity = _velocity * ( 1.0 - _velocity_cov ) + updated_velocity * _velocity_cov;

	_position = updated_position;

	size_t observedSlices = observation.sliceRadii.size();
	int oldSize = (int) _radii.size();
	if (observedSlices >= oldSize ) { //new observation contains more slices, merge up to old size and add newly observed slices
	
		_radii.resize( observedSlices ); 
		for ( int i = 0; i < oldSize; i++ ) {
			if (observation.sliceRadii[i] > 0.0) { 
				if ( age < 100 ) {
					_radii[i] = ( age * _radii[i] + (100 - age) * observation.sliceRadii[i] ) / 100.0;
				} else if (age < 250) {
					_radii[i] = ( age * _radii[i] + observation.sliceRadii[i] ) / (age + 1);
				}	
			}
		}
		for ( int j = oldSize; j < observedSlices; j++ ) {
			if (observation.sliceRadii[j] > 0.0) {
				_radii[j] = observation.sliceRadii[j];
			} else {
				_radii[j] = pow( 0.5, j -oldSize) * _radii[oldSize];
			}
		}
		
	} else { //new observation contains less slices, only update those contained and leave the others as before
	  
		for ( int i = 0; i < observedSlices; i++ ) {
			if (observation.sliceRadii[i] > 0.0) { 
				if ( age < 2 ) {
					_radii[i] = observation.sliceRadii[i];
				} else if ( age < 100 ) {
					_radii[i] = ( age * _radii[i] + (100.0 - age) * observation.sliceRadii[i] ) / 100.0;
				} else if (age < 250) {
					_radii[i] = ( age * _radii[i] + observation.sliceRadii[i] ) / (age + 1);
				}	
			}
		}
	  
	}
	
	
	//wallThickness update
	if ( observation.wallThickness.size() < 2 ) { //object is solid, no update on wallThickness
		if ( (_wallThickness.size() > 2) && (unobserved > 2) ) {
			ROS_WARN("An object that was previously known as hollow was now detected as solid");
			_wallThickness.resize(1);
			_wallThickness.at(0) = 0.0;
		}
		
	  
	} else if ( observedSlices >= oldSize ) { //new observation contains more slices, merge up to old size and add newly observed slices

		_wallThickness.resize( observedSlices );
		for ( int i = 0; i < oldSize; i++ ) {
			if (observation.wallThickness[i] > 0.0) {
				if ( age < 100 ) {
					_wallThickness[i] = ( age * _wallThickness[i] + (100 - age) * observation.wallThickness[i] ) / 100.0;
				} else if (age < 250) {
					_wallThickness[i] = ( age * _wallThickness[i] + observation.wallThickness[i] ) / (age + 1);
				}
			}
		}
		for ( int j = oldSize; j < observedSlices; j++ ) {
			if (observation.wallThickness[j] > 0.0) {
				_wallThickness[j] = observation.wallThickness[j];
			} else {
				_wallThickness[j] = _wallThickness[oldSize];
			}
		}
		
	} else { //new observation contains less slices, only update those contained and leave the others as before
	  
		for ( int i = 0; i < observedSlices; i++ ) {
			if (observation.wallThickness[i] > 0.0) {
				if ( age < 2) {
					_wallThickness[i] = observation.wallThickness[i];
				} else if ( age < 100 ) {
					_wallThickness[i] = ( age * _wallThickness[i] + (100.0 - age) * observation.wallThickness[i] ) / 100.0;
				} else if (age < 250) {
					_wallThickness[i] = ( age * _wallThickness[i] + observation.wallThickness[i] ) / (age + 1);
				}
			}
		}
	  
	}
	
	//handleHull update if available
	
	if (observation.handleHull.size() > 1) {
		if (_handleHull.size() == 0) { 
			_handleHull = observation.handleHull;
		} else if ( observation.handleHull.size() > 0.7 * _handleHull.size() ) {
			_handleHull = observation.handleHull;
		}
	}
	_last_time = observation.time;

	unobserved = 0;
        ++age;
}//Tracked_object::update_position

void Tracked_object::update_object ( pcl::PointCloud<pcl::PointXYZRGB> newObject , Eigen::Vector3f objPosition, double additionalDegrees) {
	pcl::PointCloud<pcl::PointXYZRGB> constructedAddition;
	pcl::copyPointCloud(newObject, constructedAddition);
	constructedObject = newObject;
	degreesReconstructed += additionalDegrees;

	_last_object_position = objPosition;
}//Tracked_object::update_object

std::vector<double> Tracked_object::get_radii () {
	return _radii;
}//Tracked_object::get_radius

std::vector<double> Tracked_object::get_wallThickness () {
	return _wallThickness;
}//Tracked_object::get_wallThickness

double Tracked_object::get_interval ( const ros::Time time) {
	if (age > 0) {
		return ( time.toSec() - _first_time.toSec() ) / age;
	} else {
		return 0;
	}

}//Tracked_object::get_interval

std::vector<Eigen::Vector3f> Tracked_object::get_handleHull () {
	return _handleHull;
}//Tracked_object::get_handleHull

Eigen::Vector3f Tracked_object::get_position () {
	return _position;
}//Tracked_object::get_position


pcl::PointCloud<pcl::PointXYZRGB> Tracked_object::getObject () {
	return constructedObject;

}//Tracked_object::getObject

Eigen::Vector3f Tracked_object::getLastObjectPosition () {
	return _last_object_position;
}//Tracked_object::getLastObjectPosition

double Tracked_object::getDegreesReconstructed () {
	return degreesReconstructed;
}//Tracked_object::getDegreesReconstructed

double Tracked_object::getHeight () {
	return ( _height + _position[2] );
}//Tracked_object::getHeight


}//namespace Conveyor_object_tracking
