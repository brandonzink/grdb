#include <fcntl.h>
#include <math.h>
#include <limits.h>
#include "graph.h"
#include "cli.h"


int component_sssp(component_t c,vertexid_t vertex_1,vertexid_t vertex_2,int *n,int *total_weight,vertexid_t **path){

	//Total number of vertices
	int number_of_verticies(){

		off_t off;
		ssize_t len;
		char s[BUFSIZE], *buf;
		int fd, length, num_v;
		length = sizeof(vertex_t);
		num_v = 0;
		buf = malloc(length);

		//Open the file of verticies
		memset(s, 0, BUFSIZE);
		sprintf(s, "%s/%d/%d/v", grdbdir, gno, cno);
		fd = open(s, O_RDONLY);

		//Read the verticies, incriment the number of verticies variable
		for (off = 0;;off += length){
			lseek(fd, off, SEEK_SET);
			len = read(fd, buf, length);
			if (len <= 0)
				break;
			num_v += 1;
		}

		//Close file
		close(fd);

		return(num_v);
	}

	//Put a new vertex in path array
	void alter_path_array(vertexid_t **path, vertexid_t new, int *path_size){
		//reallocate room
		vertexid_t *temp_path_for_update = realloc(*path, (*path_size + 1) * sizeof(vertexid_t));
		//incriment path size counters
		if (temp_path_for_update){
			temp_path_for_update[*path_size] = new + 1;
			*path = temp_path_for_update;
			*path_size += 1;
		}
	}

	//Work backwards, get shortest path
	void find_the_path_backwards(int num_v, vertexid_t parent_array[num_v]){
		vertexid_t current, temp[num_v];
		int m = num_v - 1;

		//keeps track of location
		current = vertex_2;

		//used to hold temp path
		for (int i = 0; i < num_v; i += 1){
			temp[i] = -1;
		}

		//make the path
		while (parent_array[current] != -1){
			temp[m] = current;
			current = parent_array[current];
			m -= 1;
		}

		//update the actual path array
		for (int i = m + 1; i < num_v; i += 1){
			alter_path_array(path, temp[i], n);
		}
	}

	//Get the weight of edge given verticies as input from file
	int read_edge_weight_from_file(vertexid_t u, vertexid_t v) {
		char s[BUFSIZE];
		int fd, weight, offset;
		struct edge e;
		attribute_t attr;

		weight = -1;

		//open file that contains the edges
		memset(s, 0, BUFSIZE);
		sprintf(s, "%s/%d/%d/e", grdbdir, gno, cno);
		fd = open(s, O_RDONLY);

		//get edge from the file
		edge_init(&e);
		edge_set_vertices(&e, u, v);
		edge_read(&e, c->se, fd);

		//find the edge weight that will be returned
		if (e.tuple != NULL){
			attr = e.tuple->s->attrlist;
			offset = tuple_get_offset(e.tuple, attr->name);
			if (offset >= 0){
				weight = tuple_get_int(e.tuple->buf + offset);
			}
		}

		//close edge file
		close(fd);

		//return the weight
		return(weight);
	}

	//Initialize min_path_found_array and current_distance_array arrays
	void init_min_path_found_array_current_distance_array(int num_v, int min_path_found_array[num_v], int current_distance_array[num_v], vertexid_t parent_array[num_v], vertexid_t vertex_1){
		for (int i = 0; i < num_v; i += 1){
			if (i == (vertex_1)){
				min_path_found_array[i] = 1;
				current_distance_array[i] = 0;
			} else {
				min_path_found_array[i] = 0;
				current_distance_array[i] = INT_MAX;
			}
			parent_array[i] = -1;
		}
	}

	//Initialize adjacency_matrix and cost_matrix arrays
	void init_adjacency_matrix_cost_matrix(int num_v, int adjacency_matrix[num_v][num_v], int cost_matrix[num_v][num_v]){
		int result;
		for (int i = 0; i < num_v; i += 1){
			for (int j = 0; j < num_v; j += 1){
				result = read_edge_weight_from_file(i + 1, j + 1);
				if (result >= 0){
					adjacency_matrix[i][j] = 1;
					cost_matrix[i][j] = result;
				} else {
					adjacency_matrix[i][j] = 0;
					cost_matrix[i][j] = INT_MAX;
				}
			}
		}
	}

	//Check if a vertex v is in min_path_found_array
	int in_min_path_found_array(int num_v, int min_path_found_array[num_v], vertexid_t v){
		return min_path_found_array[v];
	}

	//Checks if all vertices in the graph have been added to min_path_found_array
	int bool_is_array_full(int num_v, int min_path_found_array[num_v]){
		int full = 1;
		for (int i = 0; i < num_v; i += 1){
			if (min_path_found_array[i] == 0)
				full = 0;
		}

		return full;
	}

	//Get the cheapest edge connected to min_path_found_array that's not already in min_path_found_array
	vertexid_t choose_min_path(int num_v,int min_path_found_array[num_v],int adjacency_matrix[num_v][num_v],int cost_matrix[num_v][num_v]){

		int min_weight = INT_MAX;
		vertexid_t new_vert, parent;

		for (vertexid_t i = 0; i < num_v; i += 1){
			if (min_path_found_array[i] == 1){   //If vertex i in min_path_found_array
				//Check costs of going to adjacent vertices not in min_path_found_array
				for (vertexid_t j = 0; j < num_v; j += 1){
					if (!in_min_path_found_array(num_v, min_path_found_array, j) && cost_matrix[i][j] < min_weight){
						min_weight = cost_matrix[i][j];
						parent = i;
						new_vert = j;
					}
				}
			}
		}

		min_path_found_array[new_vert] = 1;

		return new_vert;
	}

	void make_edges_relaxed(int num_v, vertexid_t w,
					 int adjacency_matrix[num_v][num_v],
					 int cost_matrix[num_v][num_v],
					 int current_distance_array[num_v],
					 vertexid_t parent_array[num_v])
	{
		for (int i = 0; i < num_v; i += 1){
			if (adjacency_matrix[w][i] && current_distance_array[i] > (current_distance_array[w] + cost_matrix[w][i])){
				current_distance_array[i] = current_distance_array[w] + cost_matrix[w][i];
				parent_array[i] = w;
			}
		}
	}

	vertex_1 -= 1;
	vertex_2 -= 1;
	int num_v = number_of_verticies();
	vertexid_t w, parent_array[num_v];   //parent_array = parent list
	int min_path_found_array[num_v], current_distance_array[num_v]; //min_path_found_array = min path found list, current_distance_array = Current distance list
	int adjacency_matrix[num_v][num_v], cost_matrix[num_v][num_v]; //adjacency_matrix = adjacency matrix, cost_matrix = cost matrix
	vertexid_t *temp_path_for_update = malloc(sizeof(vertexid_t));


	//Initialize path
	if (temp_path_for_update){
		*path = temp_path_for_update;
		*path[0] = vertex_1 + 1;
	}
	*n = 1;

	//Initialize matrices and lists
	init_min_path_found_array_current_distance_array(num_v, min_path_found_array, current_distance_array, parent_array, vertex_1);
	init_adjacency_matrix_cost_matrix(num_v, adjacency_matrix, cost_matrix);

	//Relax edges connected to vertex_1
	make_edges_relaxed(num_v, vertex_1, adjacency_matrix, cost_matrix, current_distance_array, parent_array);

	//Add cheapest edges and relax until all shortest paths found
	while(!bool_is_array_full(num_v, min_path_found_array)){
		w = choose_min_path(num_v, min_path_found_array, adjacency_matrix, cost_matrix);
		make_edges_relaxed(num_v, w, adjacency_matrix, cost_matrix, current_distance_array, parent_array);
	}


	find_the_path_backwards(num_v, parent_array);
	*total_weight = current_distance_array[vertex_2];


	/* Change this as needed */
	return 0;
}
