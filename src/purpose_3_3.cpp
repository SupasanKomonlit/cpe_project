// FILE			: purpose_3_3.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 16 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

boost::qvm::mat< double , 1 , 6 > mat_force_thruster = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_buoncy = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_gravity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_epsilon = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_viscosity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::mat< double , 1 , 6 > mat_force_summation = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::vec< double , 6 > vec_calculate_velocity = { 0 , 0 , 0 , 0 , 0 , 0 };
boost::qvm::vec< double , 6 > vec_calculate_acceleration = { 0 , 0 , 0 , 0 , 0 , 0 };

void calculate()
{
    tf::Quaternion temp_quaternion;
    boost::qvm::vec< double , 3 > vec_temporary;
    //Prepare data for buoncy force
    temp_quaternion = tf::Quaternion( 0 , 0 , buoncy , 0 );
    temp_quaternion = current_quaternion.inverse() * temp_quaternion * current_quaternion;
    this->vec_temporary.a[0] = temp_quaternion.x();
    this->vec_temporary.a[1] = temp_quaternion.y();
    this->vec_temporary.a[2] = temp_quaternion.z();
    std::memcpy( (void*) &this->mat_force_buoncy.a[0][0],
            (void*) this->vec_temporary.a ,
            sizeof( double ) * 3 );
    this->vec_temporary = boost::qvm::cross( this->vec_temporary , 
            zeabus:::robot::distance_center_buoncy );
    std::memcpy( (void*) &this->mat_force_buoncy.a[0][3],
            (void*) this->vec_temporary.a ,
            sizeof( double ) * 3 );
    //Prepare data for gravity
    temp_quaternion = tf::Quaternion( 0 , 0 , -1.0*weight , 0 );
    temp_quaternion = current_quaternion.inverse() * temp_quaternion * current_quaternion;
    this->vec_temporary.a[0] = temp_quaternion.x();
    this->vec_temporary.a[1] = temp_quaternion.y();
    this->vec_temporary.a[2] = temp_quaternion.z();
    std::memcpy( (void*) &this->mat_force_gravity.a[0][0],
            (void*) this->vec_temporary.a ,
            sizeof( double ) * 3 );
    this->vec_temporary = boost::qvm::cross( this->vec_temporary , 
            zeabus:::robot::distance_center_gravity );
    std::memcpy( (void*) &this->mat_force_gravity.a[0][3],
            (void*) this->vec_temporary.a ,
            sizeof( double ) * 3 );
}

void report()
{

}
