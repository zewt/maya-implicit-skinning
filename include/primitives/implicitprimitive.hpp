/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#ifndef IMPLICITPRIMITIVE_HPP
#define IMPLICITPRIMITIVE_HPP

#include "point_cu.hpp"
#include "vec3_cu.hpp"

#include "blob.hpp"
#include "cylinder.hpp"
#include "plane.hpp"
#include "hermiteRBF.hpp"

struct ImplicitPrimitive
{
    virtual float f(const Point_cu &p) const = 0;
    virtual Vec3_cu gf(const Point_cu &p) const = 0;
    virtual float fngf(Vec3_cu &gf, const Point_cu &p) const = 0;
    BBox_cu getBbox(){ return _bbox; }
    virtual void updateBbox() = 0;
    virtual ~ImplicitPrimitive() {};
protected:
    BBox_cu _bbox;
};

struct BlobPrimitive : public ImplicitPrimitive
{
    BlobPrimitive() : ImplicitPrimitive(), _blob(new S_Blob()) {
        updateBbox();
    }

    ~BlobPrimitive(){
        delete _blob;
    }

    float f(const Point_cu &p) const{
        return _blob->f(p);
    }

    Vec3_cu gf(const Point_cu &p) const {
        return _blob->gf(p);
    }

    float fngf(Vec3_cu &gf, const Point_cu &p) const {
        return _blob->fngf(gf, p);
    }

    void updateBbox(){
        // init bbox with segment points
        _bbox = BBox_cu();
        _bbox.add_point( _blob->get_pos() );
        // extend it using radius
        float radius = _blob->get_potential_radius();
        Vec3_cu extent = Vec3_cu(radius, radius, radius);
        Point_cu pmin = _bbox.pmin - extent;
        Point_cu pmax = _bbox.pmax + extent;
        _bbox = BBox_cu(pmin, pmax);
    }

    S_Blob *_blob;
};

struct CylinderPrimitive : public ImplicitPrimitive
{
    CylinderPrimitive() : ImplicitPrimitive(), _cylinder(new S_Cylinder()) {
        updateBbox();
    }

    ~CylinderPrimitive(){
        delete _cylinder;
    }

    float f(const Point_cu &p) const{
        return _cylinder->f(p);
    }

    Vec3_cu gf(const Point_cu &p) const {
        return _cylinder->gf(p);
    }

    float fngf(Vec3_cu &gf, const Point_cu &p) const {
        return _cylinder->fngf(gf, p);
    }

    void updateBbox(){
        // init bbox with segment points
        _bbox = BBox_cu(_cylinder->get_origin(), _cylinder->get_origin() + _cylinder->get_dir());
        // extend it using radius
        float radius = _cylinder->get_potential_radius();
        Vec3_cu extent = Vec3_cu(radius, radius, radius);
        Point_cu pmin = _bbox.pmin - extent;
        Point_cu pmax = _bbox.pmax + extent;
        _bbox = BBox_cu(pmin, pmax);
    }

    S_Cylinder *_cylinder;
};

struct PlanePrimitive : public ImplicitPrimitive
{
    PlanePrimitive() :
        ImplicitPrimitive(), _plane(new S_Plane(-Vec3_cu::unit_x(), Point_cu(), 1.f)) {
        updateBbox();
    }

    ~PlanePrimitive(){
        delete _plane;
    }

    float f(const Point_cu &p) const {
        return _plane->f( p );
    }

    Vec3_cu gf(const Point_cu &p) const {
        return _plane->gf( p );
    }

    float fngf(Vec3_cu &gf, const Point_cu &p) const {
        return _plane->fngf( gf, p );
    }

    void updateBbox(){
        _bbox = BBox_cu();
        _bbox.add_point(_plane->p - _plane->n.normalized()*_plane->get_radius());
        _bbox.add_point(_plane->p + _plane->n.normalized()*_plane->get_radius());
    }

    S_Plane *_plane;
};

struct HRBFPrimitive : public ImplicitPrimitive
{
    HRBFPrimitive(const std::vector<Vec3_cu> &nodes,
                  const std::vector<Vec3_cu> &n_nodes) :
        ImplicitPrimitive(), _nodes(nodes), _n_nodes(n_nodes) {
        _hrbf = new HermiteRBF;
        _hrbf->initialize();
        _hrbf->init_coeffs(_nodes, _n_nodes);
        _radius = 2.f;
        _hrbf->set_radius(_radius);
        updateBbox();
    }

    ~HRBFPrimitive(){
        _hrbf->clear();
        delete _hrbf;
    }

    float f(const Point_cu &p) const{
//        return _hrbf->f(p);
        return 0;
    }

    Vec3_cu gf(const Point_cu &p) const {
//        return _hrbf->gf(p);
        return Vec3_cu();
    }

    float fngf(Vec3_cu &gf, const Point_cu &p) const {
//        return _hrbf->fngf(gf, p);
        gf = Vec3_cu();
        return 0;
    }

    void updateBbox(){
        _bbox = BBox_cu();
        // initialize bbox with hrbf points
        for (unsigned int i=0; i<_nodes.size(); ++i){
            _bbox.add_point(_nodes[i].to_point());
        }
        // resize bbox according to radius
        Point_cu pmin = _bbox.pmin;
        Point_cu pmax = _bbox.pmax;
        _bbox.add_point( pmin + Point_cu(_radius, _radius, _radius) );
        _bbox.add_point( pmax + Point_cu(-_radius, -_radius, -_radius) );
    }

    void update_hrbf(){
        assert( _nodes.size() == _n_nodes.size() );

        _hrbf->init_coeffs(_nodes, _n_nodes);
        _hrbf->set_radius(_radius);
    }

    void set_radius(float r){
        _radius = r;
        _hrbf->set_radius(_radius);
    }

    void add_sample_to_selection( int s ){
        // Check for doubles
        unsigned int i = 0;
        for(; i<_selected_samples.size() && _selected_samples[i] != s; ++i);
        // if not already registered => register s
        if (i >= _selected_samples.size())
            _selected_samples.push_back( s );
    }

    void remove_sample_from_selection( int s ){
        unsigned int i = 0;
        for(; i<_selected_samples.size() && _selected_samples[i] == s; ++i);
        // if not already registered => register s
        if (i < _selected_samples.size())
            _selected_samples.erase( _selected_samples.begin() + i );
    }

    void clean_samples_selection(){
        _selected_samples.clear();
    }

    void transform_selected( const Transfo &tr) {
        for (unsigned int i=0; i<_selected_samples.size(); ++i){
            _nodes[ _selected_samples[i] ] = tr * _nodes[ _selected_samples[i] ].to_point();
            _n_nodes[ _selected_samples[i] ] = tr * _n_nodes[ _selected_samples[i] ];
        }
    }

    HermiteRBF *_hrbf;
    float _radius;
    std::vector<Vec3_cu> _nodes;
    std::vector<Vec3_cu> _n_nodes;

    std::vector< int > _selected_samples;
};

#endif // IMPLICITPRIMITIVE_HPP
