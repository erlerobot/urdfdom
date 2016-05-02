/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Alejandro Herrnández */

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/hros.h>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <tinyxml.h>
#include <console_bridge/console.h>

namespace urdf{

	bool parseCognition(HROSCognition &cognition, TiXmlElement* config)
	{
		cognition.clear();

		// Get Joint Name
		const char *name = config->Attribute("name");
		if (!name){
			CONSOLE_BRIDGE_logError("unnamed joint found");
			return false;
		}
		cognition.name = name;

		// Get Parent Link
		TiXmlElement *parent_xml = config->FirstChildElement("parent");
		if (parent_xml){
		    const char *pname = parent_xml->Attribute("link");
		    if (!pname){
		      CONSOLE_BRIDGE_logInform("no parent link name specified for Cognition link [%s]. this might be the root?", cognition.name.c_str());
		    }else{
		      cognition.parent_link_name = std::string(pname);
		    }
		}

		return true;
	}

	bool exportHROSCognition(HROSCognition &cognition, TiXmlElement* xml)
	{
		TiXmlElement * cognition_xml = new TiXmlElement("cognition");
		cognition_xml->SetAttribute("name", cognition.name);
		// parent 
		TiXmlElement * parent_xml = new TiXmlElement("parent");
		parent_xml->SetAttribute("link", cognition.parent_link_name);
		cognition_xml->LinkEndChild(parent_xml);
		xml->LinkEndChild(cognition_xml);
	}

	bool addCognition(ModelInterfaceSharedPtr &model, std::string name, std::string parent)
	{
	  urdf::HROSCognitionSharedPtr cognition;
	  cognition.reset(new urdf::HROSCognition);
	  cognition->clear();
	  cognition->name = name;
	  cognition->parent_link_name = parent;

	  model->cognitions_.insert(std::make_pair(cognition->name, cognition));
	}

	bool removeCognition(ModelInterfaceSharedPtr &model, std::string name)
	{
		model->cognitions_.erase(name);	
	}

	bool parseCommunication(HROSCommunication &comm, TiXmlElement* config)
	{
		comm.clear();

		// Get Joint Name
		const char *name = config->Attribute("name");
		if (!name){
			CONSOLE_BRIDGE_logError("unnamed joint found");
			return false;
		}
		comm.name = name;

		// Get Parent Link
		TiXmlElement *parent_xml = config->FirstChildElement("parent");
		if (parent_xml){
		    const char *pname = parent_xml->Attribute("link");
		    if (!pname){
		      CONSOLE_BRIDGE_logInform("no parent link name specified for Cognition link [%s]. this might be the root?", comm.name.c_str());
		    }else{
		      comm.parent_link_name = std::string(pname);
		    }
		}

		return true;
	}

	bool exportHROSCommunication(HROSCommunication &comm, TiXmlElement* xml)
	{
		TiXmlElement * comm_xml = new TiXmlElement("communication");
  		comm_xml->SetAttribute("name", comm.name);
		// parent 
		TiXmlElement * parent_xml = new TiXmlElement("parent");
		parent_xml->SetAttribute("link", comm.parent_link_name);
		comm_xml->LinkEndChild(parent_xml);
		xml->LinkEndChild(comm_xml);
	}

	bool addCommunication(ModelInterfaceSharedPtr &model, std::string name, std::string parent)
	{
	  urdf::HROSCommunicationSharedPtr comm;
	  comm.reset(new urdf::HROSCommunication);
	  comm->clear();
	  comm->name = name;
	  comm->parent_link_name = parent;

	  model->communications_.insert(std::make_pair(comm->name, comm));
	}

	bool removeCommunication(ModelInterfaceSharedPtr &model, std::string name)
	{
		model->communications_.erase(name);	
	}
}