/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
  
#include "kdl_codyco/treeserialization.hpp"
#include <kdl/joint.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>

namespace KDL {
    
    void TreeSerialization::addDFSrecursive(SegmentMap::const_iterator current_el,int & link_cnt)
    {
        if( current_el->second.segment.getJoint().getType() != Joint::None ) {
            joints[current_el->second.q_nr] = current_el->first;
        }
        links[link_cnt] = current_el->first;
        link_cnt++;
                
        for( unsigned int i=0; i < current_el->second.children.size(); i++ ) {
            addDFSrecursive(current_el->second.children[i],link_cnt);
        }
        
    }
    
    TreeSerialization::TreeSerialization() 
    {
        links.clear();
        joints.clear();
    }

    TreeSerialization::~TreeSerialization() {}
    
    TreeSerialization::TreeSerialization(const Tree & tree) 
    {
        links.resize(tree.getNrOfSegments());
        joints.resize(tree.getNrOfJoints());
        
        
        SegmentMap::const_iterator root;
        SegmentMap::const_iterator child;
        
        int link_cnt = 0;
        
        tree.getRootSegment(root);
        for( unsigned int i=0; i < root->second.children.size(); i++ ) {
            addDFSrecursive(root->second.children[i],link_cnt);
        }
        
        assert(this->is_consistent(tree));
    }
        
    TreeSerialization::TreeSerialization(std::vector<std::string> & links_in, std::vector<std::string> & joints_in) 
    {
        links = links_in;
        joints = joints_in;
    }
    
    int TreeSerialization::getJointId(std::string joint_name)
    {
        std::vector<std::string>::iterator it;
        it = std::find(joints.begin(),joints.end(),joint_name);
        if( it != joints.end() ) {
            return it - joints.begin();
        } else {
            return -1;
        }
    }
        
    int TreeSerialization::getLinkId(std::string link_name)
    {
        std::vector<std::string>::iterator it;
        it = std::find(links.begin(),links.end(),link_name);
        if( it != links.end() ) {
            return it - links.begin();
        } else {
            return -1;
        }
    }
    
    std::string TreeSerialization::getJointName(int joint_id)
    {
        return joints[joint_id];
    }
    
    std::string TreeSerialization::getLinkName(int link_id)
    {
        return links[link_id];
    }
    
    bool TreeSerialization::is_consistent(const Tree & tree)
    {
        SegmentMap::const_iterator seg;
        if( tree.getNrOfJoints() != joints.size() || tree.getNrOfSegments() !=  links.size() ) return false;
        
        
        unsigned int i;
        
        for(i = 0; i < links.size(); i++ ) {
            if( !tree.getSegment(links[i],seg) ) return false;
        }
        
        
        for(i = 0; i < joints.size(); i++ ) {
            if( !tree.getSegment(joints[i],seg) ) return false;
            if( seg->second.segment.getJoint().getType() == Joint::None ) return false;

        }
        return true;
        
    }
            
    int TreeSerialization::getNrOfSegments() 
    {
        return links.size();
    }
        
    int TreeSerialization::getNrOfJoints()
    {
        return joints.size();
    }
    
    bool TreeSerialization::serialize(const Tree & tree,
                                      std::vector< int> mu_root, //set of childrens of root
                                      std::vector< std::vector<int> > mu, //array of sets of childrens of each segment
                                      std::vector< int > lambda, //array of parent of each segment
                                      std::vector< int> link2joint, //array mapping 
                                      std::vector< int > recursion_order, //Visiting order for the tree, such that a parent is visited before any of his childrens
                                      std::vector<SegmentMap::const_iterator> seg_vector //array of mapping between link index and SegmentMap iterators
                                      )
    {
        //assuming that *this and tree are consistent
        //the deprecated method is more efficient
        const SegmentMap& sm = tree.getSegments();
        
        mu_root.resize(0);
        mu.resize(tree.getNrOfSegments(),std::vector< int >(0));
        lambda.resize(tree.getNrOfSegments());
        
        link2joint.resize(tree.getNrOfSegments(),tree.getNrOfSegments());
                
        seg_vector.resize(tree.getNrOfSegments());
        
        
        //create necessary vectors
        SegmentMap::const_iterator root, i;
        
        tree.getRootSegment(root);
        for( unsigned int j=0; j < root->second.children.size(); j++ ) {
            mu_root.push_back(this->getLinkId(root->second.children[j]->first));
        }
        
        for( SegmentMap::const_iterator i=sm.begin(); i!=sm.end(); ++i ) {
            if( i != root ) {
                unsigned int i_index = this->getLinkId(i->first);
                seg_vector[i_index] = i;
                
                for( unsigned int j=0; j < i->second.children.size(); j++ ) {
                    mu[i_index].push_back(this->getLinkId(i->second.children[j]->first));
                }
                
                if( i->second.segment.getJoint().getType() != Joint::None ) {
                    link2joint[i_index] = this->getJointId(i->first);
                }
                
                if( i->second.parent == root ) {
                    lambda[i_index] = -1;
                } else {
                    lambda[i_index] = this->getLinkId(i->second.parent->first);
                }
                
            }
		}
        
        //As the order of the recursion is the same, it is calculated only at configuration
        std::vector<unsigned int> index_stack;
        
        index_stack.reserve(tree.getNrOfSegments());
        recursion_order.reserve(tree.getNrOfSegments());
        
        index_stack.clear();
        recursion_order.clear();
        
        for( unsigned int j=0; j < mu_root.size(); j++ ) {
            index_stack.push_back(mu_root[j]);
        }
        
        while( !index_stack.empty() ) {
            
            unsigned int curr_index = index_stack.back();
            index_stack.pop_back();
            
            recursion_order.push_back(curr_index);
            
            //Doing the recursion on the children
            for( unsigned int j=0; j < mu[curr_index].size(); j++ ) {
                index_stack.push_back(mu[curr_index][j]);
            }
        }
        
        assert(recursion_order.size() == tree.getNrOfSegments());
        
        return true;
        
    }

}
