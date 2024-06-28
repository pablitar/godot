/**************************************************************************/
/*  nav_region.cpp                                                        */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "nav_region.h"

#include "nav_map.h"

void NavRegion::set_map(NavMap *p_map) {
	map = p_map;
	polygons_dirty = true;
	if (!map) {
		connections.clear();
	}
}

void NavRegion::set_navigation_layers(uint32_t p_navigation_layers) {
	navigation_layers = p_navigation_layers;
}

uint32_t NavRegion::get_navigation_layers() const {
	return navigation_layers;
}

void NavRegion::set_transform(Transform p_transform) {
	transform = p_transform;
	polygons_dirty = true;
}

void NavRegion::set_mesh(Ref<NavigationMesh> p_mesh) {
	mesh = p_mesh;
	polygons_dirty = true;
}

int NavRegion::get_connections_count() const {
	if (!map) {
		return 0;
	}
	return connections.size();
}

Vector3 NavRegion::get_connection_pathway_start(int p_connection_id) const {
	ERR_FAIL_COND_V(!map, Vector3());
	ERR_FAIL_INDEX_V(p_connection_id, connections.size(), Vector3());
	return connections[p_connection_id].pathway_start;
}

Vector3 NavRegion::get_connection_pathway_end(int p_connection_id) const {
	ERR_FAIL_COND_V(!map, Vector3());
	ERR_FAIL_INDEX_V(p_connection_id, connections.size(), Vector3());
	return connections[p_connection_id].pathway_end;
}

bool NavRegion::sync() {
	bool something_changed = polygons_dirty /* || something_dirty? */;

	update_polygons();

	return something_changed;
}

void NavRegion::update_polygons() {
	if (!polygons_dirty) {
		return;
	}
	polygons.clear();
	polygons_dirty = false;

	if (map == nullptr) {
		return;
	}

	if (mesh.is_null()) {
		return;
	}

	PoolVector<Vector3> vertices = mesh->get_vertices();
	int len = vertices.size();
	if (len == 0) {
		return;
	}

	PoolVector<Vector3>::Read vertices_r = vertices.read();

	polygons.resize(mesh->get_polygon_count());

	// Build
	for (size_t i(0); i < polygons.size(); i++) {
		gd::Polygon &p = polygons[i];
		p.owner = this;

		Vector<int> mesh_poly = mesh->get_polygon(i);
		const int *indices = mesh_poly.ptr();
		bool valid(true);
		p.points.resize(mesh_poly.size());
		p.edges.resize(mesh_poly.size());

		Vector3 center;
		float sum(0);

		for (int j(0); j < mesh_poly.size(); j++) {
			int idx = indices[j];
			if (idx < 0 || idx >= len) {
				valid = false;
				break;
			}

			Vector3 point_position = transform.xform(vertices_r[idx]);
			p.points[j].pos = point_position;
			p.points[j].key = map->get_point_key(point_position);

			center += point_position; // Composing the center of the polygon

			if (j >= 2) {
				Vector3 epa = transform.xform(vertices_r[indices[j - 2]]);
				Vector3 epb = transform.xform(vertices_r[indices[j - 1]]);

				sum += map->get_up().dot((epb - epa).cross(point_position - epa));
			}
		}

		if (!valid) {
			ERR_BREAK_MSG(!valid, "The navigation mesh set in this region is not valid!");
		}

		p.clockwise = sum > 0;
		if (mesh_poly.size() != 0) {
			p.center = center / float(mesh_poly.size());
		}
	}
}

NavRegion * NavRegion::duplicate_for_sync() {
	NavRegion * dup = memnew(NavRegion);
	dup->set_self(get_self());
	dup->map = map;
	dup->transform = transform;
	dup->mesh = mesh;
	dup->navigation_layers = navigation_layers;
	dup->enter_cost = enter_cost;
	dup->travel_cost = travel_cost;
	
	dup->polygons_dirty = polygons_dirty;

	dup->polygons.resize(polygons.size());
	for (size_t i = 0; i < polygons.size(); i++) {
		dup->polygons[i] = polygons[i];
		dup->polygons[i].owner = dup;
	}

	//Connections are not duplicated because it's expensive to update polygon references. 
	//Plus, they are to be generated again during sync.
	
	return dup;
}

void NavRegion::copy_polygons_and_connections(NavRegion * other_region) {
	Map<gd::Polygon *, gd::Polygon *> pointer_mappings;

	polygons_dirty = other_region->polygons_dirty;

	polygons = other_region->polygons;

	for (size_t i = 0; i < polygons.size(); i++)
	{
		polygons[i].owner = this;
		pointer_mappings[&other_region->polygons[i]] = &polygons[i];
	}

	for (size_t i = 0; i < polygons.size(); i++)
	{
		for (size_t j = 0; j < polygons[i].edges.size(); j++)
		{
			for (size_t k = 0; k < polygons[i].edges[j].connections.size(); k++)
			{
				gd::Edge::Connection connection = polygons[i].edges[j].connections[k];
				connection.polygon = pointer_mappings[connection.polygon];
				polygons[i].edges[j].connections.set(k, connection);
			}
		}
	}

	connections = other_region->connections;

	for (size_t i = 0; i < connections.size(); i++)
	{
		gd::Edge::Connection connection = connections[i];
		connection.polygon = pointer_mappings[connection.polygon];
		connections.set(i, connection);
	}
}


NavRegion::NavRegion() {}

NavRegion::~NavRegion() {}
