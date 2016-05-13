//
//  RevoluteChains.h
//  Box2D
//
//  Created by Jacob Chen on 3/27/16.
//
//

#ifndef RevoluteChains_h
#define RevoluteChains_h

# define PI 3.14159265358979323846  /* pi */

#include <vector>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

/* UNIX */
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>

const float DEG_TO_RAD = PI/180.0f;
const float32 SPEED_EPSILON = 0.0001;
const float32 ANGLE_EPSILON = 0.01;
const std::string DATA_DIR = "Revolute Chains Dir";
const std::string FILE_NAME = "Session_";

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

class RevoluteChains : public Test
{

	enum state {
		initial_state,
		moving_to_initial_state,
		randomize_state,
		moving_to_random_state,
		collecting
	};

	int number_of_sleeping_bodies = 0;
	long step_counter = 0;
    int session = 0;

public:
    
    std::vector<b2RevoluteJoint *> joints;
	std::vector<b2Body *> bodies;

	state curr_state;
    std::ofstream data_file;
    
    
    RevoluteChains()
    {
        m_world->SetGravity(b2Vec2(0.0f,-9.8f));
        {
            //body and fixture defs are common to all chain links
            b2BodyDef bodyDef;
            bodyDef.type = b2_dynamicBody;
			//bodyDef.type = b2_staticBody;
			bodyDef.angularDamping = 0.8f;
            bodyDef.position.Set(0,8);
            // Fixture Def
            b2FixtureDef fixtureDef;
            fixtureDef.density = 20;
            // Shape
            b2PolygonShape polygonShape;
            polygonShape.SetAsBox(5,2);
            fixtureDef.shape = &polygonShape;
            
            //create first link
            b2Body* link = m_world->CreateBody( &bodyDef );
            link->CreateFixture( &fixtureDef );
            
            //set up the common properties of the joint before entering the loop
            b2RevoluteJointDef revoluteJointDef;

            revoluteJointDef.localAnchorA.Set( 4.75,0);
            revoluteJointDef.localAnchorB.Set(-4.75,0);
			revoluteJointDef.maxMotorTorque = 100;
			revoluteJointDef.motorSpeed = 20 * DEG_TO_RAD;
            
            //use same definitions to create multiple bodies
            for (long i = 0; i < 2; i++) {
				// Create each link in chain
                b2Body* newLink = m_world->CreateBody( &bodyDef );
                newLink->CreateFixture( &fixtureDef );
				bodies.push_back(newLink);
                //...joint creation will go here...
                //inside the loop, only need to change the bodies to be joined
                revoluteJointDef.bodyA = link;
                revoluteJointDef.bodyB = newLink;
				b2RevoluteJoint* joint = (b2RevoluteJoint *)m_world->CreateJoint(&revoluteJointDef);
				joint->SetUserData((void*)(i+2));
                joints.push_back(joint);
                link = newLink;//prepare for next iteration
            }
            
            // Create circle fixture that is denser than the chains (?)
            b2FixtureDef circleFixtureDef;
            circleFixtureDef.density = 200.0f;
            
            //body with circle fixture
            b2CircleShape circleShape;
            circleShape.m_radius = 3;
            circleFixtureDef.shape = &circleShape;
            b2Body* chainBase = m_world->CreateBody( &bodyDef );
            chainBase->CreateFixture(&circleFixtureDef);
            
            //a revolute joint to connect the circle to the ground
            revoluteJointDef.bodyA = m_groundBody;//provided by testbed
            revoluteJointDef.bodyB = chainBase;
            revoluteJointDef.localAnchorA.Set(0,30);//world coords, because m_groundBody is at (0,0)
            revoluteJointDef.localAnchorB.Set(0,0);//center of circle
            b2RevoluteJoint* ground_joint = (b2RevoluteJoint*) m_world->CreateJoint( &revoluteJointDef );
			ground_joint->SetUserData((void*)0);
			joints.push_back(ground_joint);
            
            //another revolute joint to connect the chain to the circle
            revoluteJointDef.bodyA = link;//the last added link of the chain
            revoluteJointDef.bodyB = chainBase;
            revoluteJointDef.localAnchorA.Set(4.75,0);//the regular position for chain link joints, as above
            revoluteJointDef.localAnchorB.Set(0,0);//a little in from the edge of the circle
			b2RevoluteJoint *base_joint = (b2RevoluteJoint *)m_world->CreateJoint(&revoluteJointDef);
			base_joint->SetUserData((void*)1);
			joints.push_back(base_joint);

			std::printf("Size of joints = %lu \n", joints.size());

            srand((uint8)time(NULL)); // Get random states every time we run
            
            // Create the data directory if not exist
            Create_Dir(DATA_DIR);
            
            std::string current_dir = Get_Current_Working_Directory();
            
			curr_state = initial_state;

        }
        
    }
    
    std::string Get_Current_Working_Directory() {
        /* UNIX */
        char buffer[1000];
        char *answer = getcwd(buffer, sizeof(buffer));
        std::string s_cwd;
        if (answer)
        {
            s_cwd = answer;
            std::printf("Current working directory = %s\n", s_cwd.c_str());
        }
        
        return s_cwd;
        
    }

    int Create_Dir(std::string dir_path) {
        // Check if exist
        DIR *dir;
        struct dirent *dp = NULL;
        int number_of_files_in_directory = 0;
        if((dir = opendir(dir_path.c_str())) == NULL) {
            // Create directiory
            if(mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0 ) {
                return 0; // Success
            } else {
                char buffer[256];
                strerror_r(errno, buffer, 256);
                std::printf("%s\n", buffer);
                return -1;
            }
        }
        // Print whats in the directory
        
        if ((dp = readdir(dir)) != NULL) {
            while (dp != NULL) {
                errno = 0;
                std::string file_name = dp->d_name;
                std::printf("found %s\n", file_name.c_str());
                std::string extension = file_name.substr(file_name.find_last_of(".") + 1);
                if (extension == "csv") {
                    number_of_files_in_directory ++;
                }
                dp = readdir(dir);
            }
            if (errno != 0)
                perror("error reading directory");
        }
        
        session = number_of_files_in_directory;
        
        closedir(dir);
        
        return 0;
    }
    
	void Initial_State() {

		for (auto& it : joints) {
			b2RevoluteJoint * joint = (b2RevoluteJoint *)it;
			long id = (long)joint->GetUserData();
			switch (id) {
			case 0:
				// set upper limit to 90 degrees
				std::printf("setting ground joint limit \n");
				joint->SetLimits(0 * DEG_TO_RAD, 0 * DEG_TO_RAD);
				break;
                case 1:
                    std::printf("Setting base joint limit \n");
                    joint->SetLimits(0 * DEG_TO_RAD, 0 * DEG_TO_RAD);
                    break;
			default:
				// set upper and lower limit to 0
				std::printf("setting link joint limit to 0 0\n");
				joint->SetLimits(0, 0);
				break;
			}

			joint->EnableLimit(true);
		}
	}
    
	void Damp_Bodies(float damping) {
		for (auto &it : bodies) {
			b2Body * body = (b2Body *)it;
			body->SetAngularDamping(damping);
		}
	}


	bool Check_If_Reached_State() {

		bool reached = true;

		for (auto &it : joints) {
			b2RevoluteJoint *joint = (b2RevoluteJoint *)it;
			std::printf("id = %lu, Joint angle = %f, limit = %f \n", (long)joint->GetUserData(), joint->GetJointAngle(), joint->GetUpperLimit());
            if (std::abs(joint->GetJointAngle() - joint->GetUpperLimit()) < ANGLE_EPSILON) {
				reached = reached && true;
			}
			else {
				reached = false;
				break;
			}
		}
		return reached;
	}

	bool Check_Sleeping(){
		bool allSleeping = true;
		for (auto& it : bodies) {
			b2Body * body = (b2Body *)it;
			if (body->IsAwake()){ // true means its awake (?)

				//std::printf("Angular velocity = %f \n", body->GetAngularVelocity());

                if (std::abs(body->GetAngularVelocity()) < SPEED_EPSILON) {
					
					allSleeping = allSleeping && true;
					continue;
				}
				allSleeping = false;
				break;
			}
			else {
				allSleeping = allSleeping && true;
				continue;
			}
		}
		std::printf(allSleeping ? "Bodies all sleeping.\n" : "Bodies are not all sleeping\n");
		return allSleeping;
	}

	void Randomize_Joint_Positions () {
		for (auto& it: joints) {
			b2RevoluteJoint *joint = (b2RevoluteJoint *)it;
			long joint_id = (long) joint->GetUserData();
			switch (joint_id) {
                case 0: // ground joint
                    break;
                case 1: // base joint
                    break;
                case 2: { // First Link
                    int random_degree = 90 - (rand() % 180);
                    printf("First link setting degree to %d \n", random_degree);
                    float random_radians = random_degree * DEG_TO_RAD;
                    printf("First link setting radians to %f \n", random_radians);
                    joint->SetLimits(random_radians, random_radians);
                    break;
                }
                case 3: { // Second Link
                    int random_degree = 90 - (rand() % 180);
                    printf("Second link setting degree to %d \n", random_degree);
                    float random_radians = random_degree * DEG_TO_RAD;
                    printf("Second link setting radians to %f \n", random_radians);
                    joint->SetLimits(random_radians, random_radians);
                    break;
                }
                default: {
                    int random_degree = rand() % 360;
                    joint->SetLimits(random_degree * DEG_TO_RAD, random_degree * DEG_TO_RAD);
                    break;
                }
            }
		}
	}

	void Disable_Joint_Limits() {
		for (auto &it : joints) {
			b2RevoluteJoint * joint = (b2RevoluteJoint *)it;
			joint->EnableLimit(false);
		}
	}
    
    void Enable_Joint_Limits() {
        for (auto &it : joints) {
            b2RevoluteJoint * joint = (b2RevoluteJoint *)it;
            joint->EnableLimit(true);
        }
    }

	void Get_Joint_States() {
		for (auto& it: joints) {
			b2RevoluteJoint *joint = (b2RevoluteJoint *)it;
			float angle = joint->GetJointAngle();
			float speed = joint->GetJointSpeed();
			long joint_id = (long)joint->GetUserData();
			std::printf("id = %lu, angle = %f, speed = %f\n", joint_id, angle, speed);
            Write_To_File((int)joint_id, angle, speed);
		}
	}

    void Prepare_File() {
        
        std::string file_name = DATA_DIR + "/" + FILE_NAME + patch::to_string(session) + ".csv";
        std::printf("file name = %s\n", file_name.c_str());
        data_file.open(file_name.c_str(), std::ofstream::out);
        data_file << "joint id, joint angle, joint velocity \n";
        
    }
    
    void Write_To_File(int joint_id, float angle, float velocity) {
        char buffer[1000];
        snprintf(buffer, 1000, "%d, %f, %f\n", joint_id, angle, velocity);
        data_file << buffer;
    }
    
    void Close_File() {
        data_file.close();
    }
    
    void Switch_State() {
        // Move from the current state to the next state
        switch (curr_state) {
            case initial_state:
                std::printf("Switching to moving to initial state\n");
                curr_state = moving_to_initial_state;
                break;
            case moving_to_initial_state:
                std::printf("Switching to randomize state\n");
                curr_state = randomize_state;
                break;
            case randomize_state:
                std::printf("Switching to moving to randomize state\n");
                curr_state = moving_to_random_state;
                break;
            case moving_to_random_state:
                std::printf("Switching to collecting state\n");
                curr_state = collecting;
                break;
            case collecting:
                curr_state = initial_state;
                break;
        }
    }
    
    void Step(Settings* settings)
    {
		Test::Step(settings);
		switch (curr_state) {
            case initial_state:
                session ++;
                Damp_Bodies(0.0f);
                Initial_State();
                Switch_State(); // switch to moving
                break;
            case moving_to_initial_state:
                //std::printf("Moving to initial state \n");
                if (Check_If_Reached_State()){
                    std::printf("Bodies are sleeping \n");
                    // All bodies are sleeping.
                    Switch_State(); // switch to random
                }
                else {
                    //std::printf("Not all bodies are sleeping\n");
                }
                break;
            case randomize_state:
                std::printf("Make a random state \n");
                Disable_Joint_Limits();
                Randomize_Joint_Positions();
                Enable_Joint_Limits();
                Switch_State();
                break;
            case moving_to_random_state:
                std::printf("Moving to random state \n");
                if (Check_If_Reached_State()){
                    // All bodies are sleeping.
                    step_counter = 0;
                    Damp_Bodies(0.8f);
                    Disable_Joint_Limits();
                    Prepare_File();
                    Switch_State(); // switch to random
                }
                break;
            case collecting:
                //std::printf("Collecting.\n");
                if (step_counter % 100 == 0) {
                    Get_Joint_States();
                }

                if (Check_Sleeping()) {
                    Close_File();
                    Switch_State();
                }

                break;
            }       
        
    }
    
    static Test* Create()
    {
        return new RevoluteChains;
    }
    
};

#endif /* RevoluteChains_h */
