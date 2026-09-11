#pragma once

#include "motionProfile.h"
                            

class CombinedMotionProfile : public MotionProfile {
private:
	std::vector<MotionProfile*> motionProfiles;
	                                                        
	                       
public:
	explicit CombinedMotionProfile(std::vector<MotionProfile *> motion_profiles)
		: motionProfiles(std::move(motion_profiles)) {
	}

	                         
	                     
	    

	                                                             
	                                

	                              

	                                          
	                                                   
	                                                                     
	                                            

	                                       

	                                                                                                 
	                                                              

	                        
	        

	                     
	       
	                          
	      

	                                          
	                                
	     

	                        
	    

	[[nodiscard]] QTime getDuration() const override {
		QTime totalTime = 0.0;

		for (const auto motionProfile : this->motionProfiles) {
			totalTime += motionProfile->getDuration();
		}

		return totalTime;
	}

	void addMP(MotionProfile* profile) {
		this->motionProfiles.emplace_back(profile);
	}

	double maxT() override {
		double accumulatedT = 0.0;

		for (auto motion_profile : this->motionProfiles) {
			accumulatedT += motion_profile->maxT();
		}

		return accumulatedT;
	}

	                                                       
	                                         
	    

	size_t size() {
		return this->motionProfiles.size();
	}

	~CombinedMotionProfile() override = default;
};