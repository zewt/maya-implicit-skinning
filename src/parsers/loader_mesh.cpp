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
#include "loader_mesh.hpp"

#include <QDir>

// =============================================================================
namespace Loader {
// =============================================================================

// CLASS Material ==============================================================
Material::Material() : _name(), _illum(4), _Ni(1), _Ns(10), _map_Ka(), _map_Kd(), _map_Ks(), _map_Bump(), _Bm(1) {
    _Ka[0] = _Ka[1] = _Ka[2] = _Kd[0] = _Kd[1] = _Kd[2] = _Ks[0] = _Ks[1] = _Ks[2] = 0;
    _Ka[3] = _Kd[3] = _Ks[3] = 1;
    _Tf[0] = _Tf[1] = _Tf[2] = 1;
    _illum = 4;
    _Ni = _Ns = 0.5f;
    _map_Ka = _map_Kd = _map_Ks = _map_Bump = "";
    _Bm = 0;
}

//------------------------------------------------------------------------------

Material::Material(const Material& mat) {
    _Ka[0] = mat._Ka[0]; _Ka[1] = mat._Ka[1]; _Ka[2] = mat._Ka[2]; _Ka[3] = mat._Ka[3];
    _Kd[0] = mat._Kd[0]; _Kd[1] = mat._Kd[1]; _Kd[2] = mat._Kd[2]; _Kd[3] = mat._Kd[3];
    _Ks[0] = mat._Ks[0]; _Ks[1] = mat._Ks[1]; _Ks[2] = mat._Ks[2]; _Ks[3] = mat._Ks[3];
    _Tf[0] = mat._Tf[0]; _Tf[1] = mat._Tf[1]; _Tf[2] = mat._Tf[2];
    _Ni = mat._Ni;
    _Ns = mat._Ns;
    _name = mat._name;
    _map_Ka = mat._map_Ka;
    _map_Kd = mat._map_Kd;
    _map_Ks = mat._map_Ks;
    _map_Bump = mat._map_Bump;
    _illum = mat._illum;
    _Bm = mat._Bm;
}

//------------------------------------------------------------------------------

Material::~Material() {
}

//------------------------------------------------------------------------------

void Material::set_relative_paths(const std::string& p)
{
    QDir path(QString(p.c_str()));

    _map_Kd   = path.relativeFilePath( QString(_map_Kd.  c_str()) ).toStdString();
    _map_Ka   = path.relativeFilePath( QString(_map_Ka.  c_str()) ).toStdString();
    _map_Ks   = path.relativeFilePath( QString(_map_Ks.  c_str()) ).toStdString();
    _map_Bump = path.relativeFilePath( QString(_map_Bump.c_str()) ).toStdString();
}

// END CLASS Material ==========================================================

// CLASS Abs_mesh ==============================================================

void Abs_mesh::clear(){
    _vertices.clear();
    _normals.clear();
    _texCoords.clear();
    _triangles.clear();
    _groups.clear();
    _materials.clear();
    _render_faces._tris. clear();
    _render_faces._quads.clear();
}

// END CLASS Abs_mesh ==========================================================

}// END LOADER NAMESPACE =======================================================
