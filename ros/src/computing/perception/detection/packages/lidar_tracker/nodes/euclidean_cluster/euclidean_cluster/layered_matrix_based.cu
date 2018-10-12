#include "include/euclidean_cluster.h"
#include <cuda.h>

#define NON_ATOMIC_ 1

/* Connected component labeling points at GPU block thread level.
 * Input list of points is divided into multiple smaller groups.
 * Each group of point is assigned to a block of GPU thread.
 * Each thread in a block handles one point in the group. It iterates over
 * points in the group and compare the distance between the current point A
 * and the point B it has to handle.
 *
 * If the distance between A and B is less than the threshold, then those
 * two points belong to a same connected component and the cluster_changed
 * is marked by 1.
 *
 * A synchronization is called to make sure all thread in the block finish A
 * before moving to the update phase.
 * After finishing checking cluster_changed, threads update the cluster
 * index of all points. If a thread has cluster_changed is 1, then the corresponding
 * cluster of the point it is handling is changed to the cluster of B. Otherwise
 * the original cluster of A remains unchanged.
 *
 * Another synchronization is called before all threads in the block move to
 * other points after done checking A.
 *
 * After this kernel finishes, all points in each block are labeled.
 */


/* Arrays to remember:
 * cluster_name: the index of the cluster that each point belong to
 * 				i.e. point at index i belong to cluster cluster_name[i]
 * cluster_list: the list of remaining clusters
 * cluster_location: location of the remaining clusters in the cluster list
 * 					i.e. cluster A locate in index cluster_location[A] in the
 * 					cluster_list
 * matrix: the adjacency matrix of the cluster list, each cluster is a vertex.
 * 			This matrix is rebuilt whenever some clusters are merged together
 */

/* The adjacency matrix is divided into non-overlap sub-matrices each
 * contains 1024 consecutive columns and 1024 consecutive rows. Empty
 * sub-matrices are ignored.
 *
 * A sample matrix is created to store the status of each sub-matrix.
 * A cell of it is 0 if the corresponding sub-matrix is zero and 1 otherwise.
 *
 * The sub_mat_location array is the result of a prefix sum on the sample
 * matrix. It is used to access non-zero sub-matrices.
 *
 * Finally, the actual non-zero sub-matrices are stored in a matrix,
 * which is an int array.
 */

__global__ void blockClusteringM4(float *x, float *y, float *z, int point_num, int *cluster_name, float threshold)
{
	int block_start = blockIdx.x * blockDim.x;
	int block_end = (block_start + blockDim.x > point_num) ? point_num : block_start + blockDim.x;
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];
	/* Each thread is in charge of one point in the block.*/
	int pid = threadIdx.x + block_start;
	/* Local cluster to record the change in the name of the cluster each point belong to */
	__shared__ int local_cluster_idx[BLOCK_SIZE_X];
	/* Cluster changed to check if a cluster name has changed after each comparison */
	__shared__ int cluster_changed[BLOCK_SIZE_X];

	if (pid < block_end) {
		local_cluster_idx[threadIdx.x] = threadIdx.x;
		local_x[threadIdx.x] = x[pid];
		local_y[threadIdx.x] = y[pid];
		local_z[threadIdx.x] = z[pid];
		__syncthreads();

		float cx = local_x[threadIdx.x];
		float cy = local_y[threadIdx.x];
		float cz = local_z[threadIdx.x];

		/* Iterate through all points in the block and check if the point at row index
		 * and at column index belong to the same cluster.
		 * If so, then name of the cluster of the row point is changed into the name
		 * of the cluster of column point.
		 * */
		for (int rid = 0; rid < block_end - block_start; rid++) {
			float distance = norm3df(cx - local_x[rid], cy - local_y[rid], cz - local_z[rid]);
			int row_cluster = local_cluster_idx[rid];
			int col_cluster = local_cluster_idx[threadIdx.x];

			cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (threadIdx.x > rid && distance < threshold) {
				cluster_changed[col_cluster] = 1;
			}
			__syncthreads();

			local_cluster_idx[threadIdx.x] = (cluster_changed[col_cluster] == 1) ? row_cluster : col_cluster;
			__syncthreads();
		}
		__syncthreads();

		int new_cluster = cluster_name[block_start + local_cluster_idx[threadIdx.x]];
		__syncthreads();

		cluster_name[pid] = new_cluster;
	}
}

__global__ void nonZeroSubMatrixCountM4(float *x, float *y, float *z, int point_num,
										int *cluster_name, int *cluster_location,
										int *sample_matrix, int sample_size,
										int cluster_num, float threshold)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int cpid = idx; cpid < point_num; cpid += stride) {
		float tmp_x = x[cpid];
		float tmp_y = y[cpid];
		float tmp_z = z[cpid];
		int col_cluster = cluster_name[cpid];
		int col = cluster_location[col_cluster];

		for (int rpid = blockIdx.y; rpid < cpid; rpid += gridDim.y) {
			float tmp_x2 = tmp_x - x[rpid];
			float tmp_y2 = tmp_y - y[rpid];
			float tmp_z2 = tmp_z - z[rpid];
			int row_cluster = cluster_name[rpid];
			int row = cluster_location[row_cluster];

			if (row_cluster != col_cluster && norm3df(tmp_x2, tmp_y2, tmp_z2) < threshold) {
				int sub_mat_col = col / BLOCK_SIZE_X;
				int sub_mat_row = row / BLOCK_SIZE_X;

				sample_matrix[sub_mat_col + sub_mat_row * sample_size] = 1;
			}
		}
		__syncthreads();
	}
}

__global__ void buildAdjacencyMatrixM4(float *x, float *y, float *z, int point_num,
										int *cluster_name, int *cluster_location,
										int *sub_mat_location, int sample_size,
										int cluster_num, float threshold, int *matrix)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int *sub_mat;

	// Loop over columns
	for (int cpid = idx; cpid < point_num; cpid += stride) {
		float tmp_x = x[cpid];
		float tmp_y = y[cpid];
		float tmp_z = z[cpid];
		int col_cluster = cluster_name[cpid];
		int col = cluster_location[col_cluster];

		for (int rpid = blockIdx.y; rpid < cpid; rpid += gridDim.y) {
			float tmp_x2 = tmp_x - x[rpid];
			float tmp_y2 = tmp_y - y[rpid];
			float tmp_z2 = tmp_z - z[rpid];
			int row_cluster = cluster_name[rpid];
			int row = cluster_location[row_cluster];

			if (row_cluster != col_cluster && norm3df(tmp_x2, tmp_y2, tmp_z2) < threshold) {
				// Location of the sub-matrix that contains the cell (row, col)
				int sub_mat_col = col / BLOCK_SIZE_X;
				int sub_mat_row = row / BLOCK_SIZE_X;

				sub_mat = matrix + sub_mat_location[sub_mat_col + sub_mat_row * sample_size] * BLOCK_SIZE_X * BLOCK_SIZE_X;

				// Local row and col of the cell in the sub-matrix
				int local_col = col % BLOCK_SIZE_X;
				int local_row = row % BLOCK_SIZE_X;

				sub_mat[local_col + local_row * BLOCK_SIZE_X] = 1;
			}
		}
	}
}


__global__ void mergeLocalClustersM4(int *cluster_list, int *matrix, int cluster_num, bool *changed,
									int *sample_matrix, int sample_mat_size, int *sub_mat_location)
{
	__shared__ int local_cluster_idx[BLOCK_SIZE_X];
	__shared__ int local_cluster_changed[BLOCK_SIZE_X];
	bool lchanged = false;
	__shared__ bool schanged;

	schanged = false;
	__syncthreads();

	if (blockIdx.x < sample_mat_size && sample_matrix[blockIdx.x + blockIdx.x * sample_mat_size] != 0) {
		// Access the corresponding sub-matrix
		// Column id is also the point id
		int local_col = threadIdx.x;
		int global_col = threadIdx.x + blockIdx.x * BLOCK_SIZE_X;
		int row_end = (blockIdx.x * BLOCK_SIZE_X + BLOCK_SIZE_X <= cluster_num) ? BLOCK_SIZE_X : cluster_num - blockIdx.x * BLOCK_SIZE_X;

		if (global_col < cluster_num) {
			int *sub_mat = matrix + sub_mat_location[blockIdx.x + blockIdx.x * sample_mat_size] * BLOCK_SIZE_X * BLOCK_SIZE_X;

			local_cluster_idx[threadIdx.x] = threadIdx.x;
			__syncthreads();

			for (int local_row = 0; local_row < row_end; local_row++) {
				int col_cluster = local_cluster_idx[local_col];
				int row_cluster = local_cluster_idx[local_row];

				// Reset the 'changed' status of all columns to zero
				local_cluster_changed[local_col] = 0;
				__syncthreads();

				/* If the col and row clusters are different and they are connected,
				 * then the col cluster will be 'marked' to be changed to the row cluster.
				 */
				if (local_row < local_col && col_cluster != row_cluster && sub_mat[local_col + local_row * BLOCK_SIZE_X] == 1) {
					local_cluster_changed[col_cluster] = 1;
					lchanged = true;
				}
				__syncthreads();

				/* If a col cluster X was marked 'changed', change labels of all columns which were labeled
				 * as X to the label of the row cluster
				 */
				local_cluster_idx[local_col] = (local_cluster_changed[col_cluster] == 1) ? row_cluster : col_cluster;
				__syncthreads();
			}

			// Location of the cluster name
			int new_cluster_label = cluster_list[blockIdx.x * BLOCK_SIZE_X + local_cluster_idx[local_col]];
			__syncthreads();

			cluster_list[global_col] = new_cluster_label;

			if (lchanged) {
				schanged = true;
			}
			__syncthreads();

			if (schanged && threadIdx.x == 0) {
				*changed = true;
			}
		}
	}
}


/* Merge clusters that belong to different block of threads*/
__global__ void mergeForeignClustersM4(int *matrix, int *cluster_list,
										int shift_level,
										int sub_mat_size,
										int sub_mat_offset,
										int cluster_num, bool *changed,
										int *sample_matrix, int sample_mat_size, int *sub_mat_location)
{
	// sub_mat_col_base = sub_matrix_size
	// sub_mat_row_base = 0
	int sub_mat_id = blockIdx.x / sub_mat_size;
	int sub_mat_idx = sub_mat_size + sub_mat_id * sub_mat_offset + (shift_level + blockIdx.x) % sub_mat_size;
	int sub_mat_idy = sub_mat_id * sub_mat_offset + blockIdx.x % sub_mat_size;
	bool lchanged = false;
	__shared__ bool schanged;

	__shared__ int cluster_changed[BLOCK_SIZE_X];
	__shared__ int local_clusters[BLOCK_SIZE_X];

	if (threadIdx.x == 0)
		schanged = false;
	__syncthreads();

	if (sub_mat_idx < sample_mat_size && sub_mat_idy < sample_mat_size && sample_matrix[sub_mat_idx + sub_mat_idy * sample_mat_size] == 1) {
		int local_col = threadIdx.x;
		int global_col = local_col + sub_mat_idx * BLOCK_SIZE_X;
		int row_end = (sub_mat_idy * BLOCK_SIZE_X + BLOCK_SIZE_X < cluster_num) ? BLOCK_SIZE_X : cluster_num - sub_mat_idy * BLOCK_SIZE_X;

		if (global_col < cluster_num)
			local_clusters[threadIdx.x] = threadIdx.x;
		__syncthreads();

		int *sub_mat = matrix + sub_mat_location[sub_mat_idx + sub_mat_idy * sample_mat_size] * BLOCK_SIZE_X * BLOCK_SIZE_X;

		for (int local_row = 0; local_row < row_end; local_row++) {
			int col_cluster = local_clusters[threadIdx.x];

			cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (sub_mat[local_row * BLOCK_SIZE_X + local_col] == 1) {
				cluster_changed[col_cluster % BLOCK_SIZE_X] = (col_cluster < BLOCK_SIZE_X) ? 1 : 2;
				lchanged = true;
			}
			__syncthreads();

			if ((col_cluster < BLOCK_SIZE_X && cluster_changed[col_cluster] == 1) ||
					(col_cluster >= BLOCK_SIZE_X && cluster_changed[col_cluster - BLOCK_SIZE_X] == 2)) {
				local_clusters[local_col] = local_row + BLOCK_SIZE_X;
			}
			__syncthreads();
		}

		__syncthreads();

		int new_cluster_id = local_clusters[local_col];
		int global_row = threadIdx.x + sub_mat_idy * BLOCK_SIZE_X;

		if (global_row < cluster_num)
			local_clusters[threadIdx.x] = cluster_list[global_row];
		__syncthreads();

		if (new_cluster_id >= BLOCK_SIZE_X) {
			cluster_list[global_col] = local_clusters[new_cluster_id - BLOCK_SIZE_X];
		}

		__syncthreads();

		if (lchanged)
			schanged = true;

		__syncthreads();

		if (threadIdx.x == 0 && schanged)
			*changed = true;
	}
}

__global__ void rebuildSampleMatrixM4(int *old_sample_matrix, int old_size,
									int *old_sub_mat_loc, int *old_matrix,
									int *updated_cluster_list,
									int *new_cluster_location,
									int *new_sample_matrix, int new_sample_size)
{
	for (int i = blockIdx.x; i < old_size * old_size; i += gridDim.x) {
		if (old_sample_matrix[i] > 0) {
			int col = threadIdx.x + (i % old_size) * blockDim.x;
			int new_col = new_cluster_location[updated_cluster_list[col]];

			for (int row = blockIdx.y; row < col; row += gridDim.y) {
				int new_row = new_cluster_location[updated_cluster_list[row]];
				int *old_sub_mat = old_matrix + old_sub_mat_loc[col / BLOCK_SIZE_X + (row / BLOCK_SIZE_X) * old_size] * BLOCK_SIZE_X * BLOCK_SIZE_X;

				if (new_col != new_row && old_sub_mat[(row / BLOCK_SIZE_X) * old_size + (col / BLOCK_SIZE_X)] == 1) {
					new_sample_matrix[(new_col / BLOCK_SIZE_X) + (new_row / BLOCK_SIZE_X) * new_sample_size] = 1;
				}
			}
		}
	}
}

/* Rebuild the adjacency matrix after some clusters are joined together */
__global__ void rebuildAdjacencyMatrixM4(int *old_sample_matrix, int old_size,
										int *old_sub_mat_loc, int *old_matrix,
										int *updated_cluster_list,
										int *new_cluster_location,
										int *new_sub_mat_loc, int new_size, int *new_matrix)
{
	for (int i = blockIdx.x; i < old_size * old_size; i += gridDim.x) {
		if (old_sample_matrix[i] > 0) {
			int col = threadIdx.x + (i % old_size) * blockDim.x;
			int new_col = new_cluster_location[updated_cluster_list[col]];

			for (int row = blockIdx.y; row < col; row += gridDim.y) {
				int new_row = new_cluster_location[updated_cluster_list[row]];
				int *old_sub_mat = old_matrix + old_sub_mat_loc[col / BLOCK_SIZE_X + (row / BLOCK_SIZE_X) * old_size] * BLOCK_SIZE_X * BLOCK_SIZE_X;

				if (new_col != new_row && old_sub_mat[(row / BLOCK_SIZE_X) * old_size + (col / BLOCK_SIZE_X)] == 1) {
					int new_loc = new_sub_mat_loc[(new_row / BLOCK_SIZE_X) * new_size + (new_col / BLOCK_SIZE_X)] * BLOCK_SIZE_X * BLOCK_SIZE_X;

					new_matrix[new_loc + (new_row % BLOCK_SIZE_X) * BLOCK_SIZE_X + (new_col % BLOCK_SIZE_X)] = 1;
				}
			}
		}
	}
}


/* Iterate through the list of remaining clusters and mark the corresponding
 * location on cluster location array by 1
 */
__global__ void clusterMarkM4(int *cluster_list, int *cluster_location, int cluster_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < cluster_num; i += blockDim.x * gridDim.x) {
		cluster_location[cluster_list[i]] = 1;
	}
}

/* Collect the remaining clusters */
__global__ void clusterCollectorM4(int *old_cluster_list, int *new_cluster_list, int *cluster_location, int cluster_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < cluster_num; i += blockDim.x * gridDim.x) {
		new_cluster_list[cluster_location[old_cluster_list[i]]] = old_cluster_list[i];
	}
}


/* Rename the cluster name of each point after some clusters are joined together */
__global__ void applyClusterChangedM4(int *cluster_name, int *cluster_list, int *cluster_location, int point_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < point_num; i += blockDim.x * gridDim.x) {
		int old_cluster = cluster_name[i];

		cluster_name[i] = cluster_list[cluster_location[old_cluster]];
	}
}


void GpuEuclideanCluster2::extractClusters4()
{
	struct timeval start, end;

	// Initialize names of clusters
	initClusters();

	bool *check;
	bool hcheck = false;

	checkCudaErrors(cudaMalloc(&check, sizeof(bool)));
	checkCudaErrors(cudaMemcpy(check, &hcheck, sizeof(bool), cudaMemcpyHostToDevice));

	int block_x, grid_x;

	block_x = (point_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num_;
	grid_x = (point_num_ - 1) / block_x + 1;

	gettimeofday(&start, NULL);
	// Divide points into blocks of points and clustering points inside each block
	blockClusteringM4<<<grid_x, block_x>>>(x_, y_, z_, point_num_, cluster_name_, threshold_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());
	gettimeofday(&end, NULL);

	std::cout << "blockClustering = " << timeDiff(start, end) << std::endl;

	// Collect the remaining clusters
	// Locations of clusters in the cluster list
	int *cluster_location;

	gettimeofday(&start, NULL);
	checkCudaErrors(cudaMalloc(&cluster_location, sizeof(int) * (point_num_ + 1)));
	checkCudaErrors(cudaMemset(cluster_location, 0, sizeof(int) * (point_num_ + 1)));
	clusterMarkM4<<<grid_x, block_x>>>(cluster_name_, cluster_location, point_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	int current_cluster_num = 0;
	exclusiveScan(cluster_location, point_num_ + 1, &current_cluster_num);

	int *cluster_list;

	checkCudaErrors(cudaMalloc(&cluster_list, sizeof(int) * current_cluster_num));

	clusterCollectorM4<<<grid_x, block_x>>>(cluster_name_, cluster_list, cluster_location, point_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	cluster_num_ = current_cluster_num;

	dim3 grid_size, block_size;

	block_size.x = block_x;
	block_size.y = block_size.z = 1;
	grid_size.x = grid_x;
	grid_size.y = (cluster_num_ > GRID_SIZE_Y) ? GRID_SIZE_Y : cluster_num_;
	grid_size.z = 1;

	// Sample matrix to record the status of each sub-matrix in the big adjacency matrix
	int *sample_matrix;
	int sample_size;

	sample_size = (cluster_num_ - 1) / BLOCK_SIZE_X + 1;
	checkCudaErrors(cudaMalloc(&sample_matrix, sizeof(int) * sample_size * sample_size));
	checkCudaErrors(cudaMemset(sample_matrix, 0, sizeof(int) * sample_size * sample_size));

	nonZeroSubMatrixCountM4<<<grid_size, block_size>>>(x_, y_, z_, point_num_,
														cluster_name_, cluster_location,
														sample_matrix, sample_size,
														cluster_num_, threshold_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	// Prefix Sum on sample matrix to produce sub-matrix locations
	int *sub_mat_loc;

	checkCudaErrors(cudaMalloc(&sub_mat_loc, sizeof(int) * (sample_size * sample_size + 1)));
	checkCudaErrors(cudaMemcpy(sub_mat_loc, sample_matrix, sizeof(int) * sample_size * sample_size, cudaMemcpyDeviceToDevice));

	int sub_mat_num;

	exclusiveScan(sub_mat_loc, sample_size * sample_size + 1, &sub_mat_num);

	int *matrix;

	checkCudaErrors(cudaMalloc(&matrix, sizeof(int) * sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X));
	checkCudaErrors(cudaMemset(matrix, 0, sizeof(int) * sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X));


	buildAdjacencyMatrixM4<<<grid_size, block_size>>>(x_, y_, z_, point_num_,
													cluster_name_, cluster_location,
													sub_mat_loc, sample_size,
													cluster_num_, threshold_, matrix);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());
	gettimeofday(&end, NULL);

	std::cout << "Build RC and Matrix = " << timeDiff(start, end) << std::endl;

	gettimeofday(&start, NULL);
	int itr = 0;

	std::cout << "Cluster num = " << cluster_num_ << std::endl;
	int *matrix_test = (int*)malloc(sizeof(int) * sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X);

	checkCudaErrors(cudaMemcpy(matrix_test, matrix, sizeof(int) * sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X, cudaMemcpyDeviceToHost));

	for (int i = 0; i < sub_mat_num; i++) {
		for (int j = 0; j < BLOCK_SIZE_X; j++) {
			for (int k = 0; k < BLOCK_SIZE_X; k++) {
				if (matrix_test[i * BLOCK_SIZE_X * BLOCK_SIZE_X + j * BLOCK_SIZE_X + k] != 0)
					std::cout << "(" << j << "," << k << ") ";
			}
		}

		std::cout << std::endl << std::endl;
	}

	free(matrix_test);

	do {
		hcheck = false;

		checkCudaErrors(cudaMemcpy(check, &hcheck, sizeof(bool), cudaMemcpyHostToDevice));

		int block_x2, grid_x2, grid_y2;

		block_x2 = (cluster_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : cluster_num_;
		grid_x2 = (cluster_num_ - 1) / block_x2 + 1;

		mergeLocalClustersM4<<<grid_x2, block_x2>>>(cluster_list, matrix, cluster_num_, check,
													sample_matrix, sample_size, sub_mat_loc);

		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());

		int sub_matrix_size = 1;
		int sub_matrix_offset = 2;

		checkCudaErrors(cudaMemcpy(&hcheck, check, sizeof(bool), cudaMemcpyDeviceToHost));


		while (!(hcheck) && sub_matrix_size < cluster_num_ && cluster_num_ > BLOCK_SIZE_X) {

			int sub_matrix_num = (cluster_num_ - 1) / sub_matrix_offset + 1;
			block_x2 = BLOCK_SIZE_X;
			grid_x2 = sub_matrix_size * sub_matrix_num;
			grid_y2 = sub_matrix_size;

			block_size.x = block_x2;
			block_size.y = block_size.z = 1;
			grid_size.x = grid_x2;
			grid_size.y = grid_y2;
			grid_size.z = 1;

			for (int shift_level = 0; shift_level < sub_matrix_size && !(hcheck); shift_level++) {
				mergeForeignClustersM4<<<block_x2, grid_x2>>>(matrix, cluster_list,
															shift_level,
															sub_matrix_size,
															sub_matrix_offset,
															cluster_num_, check,
															sample_matrix, sample_size, sub_mat_loc);
				checkCudaErrors(cudaGetLastError());
				checkCudaErrors(cudaDeviceSynchronize());

				checkCudaErrors(cudaMemcpy(&hcheck, check, sizeof(bool), cudaMemcpyDeviceToHost));

			}

			sub_matrix_size *= 2;
			sub_matrix_offset *= 2;
		}


		/* If some changes in the cluster list are recorded (some clusters are merged together),
		 * rebuild the matrix, the cluster location, and apply those changes to the cluster_name array
		 */

		if (hcheck) {
			// Apply changes to the cluster_name array
			applyClusterChangedM4<<<grid_x, block_x>>>(cluster_name_, cluster_list, cluster_location, point_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			checkCudaErrors(cudaMemset(cluster_location, 0, sizeof(int) * (point_num_ + 1)));

			block_x2 = (cluster_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : cluster_num_;
			grid_x2 = (cluster_num_ - 1) / block_x2 + 1;

			// Remake the cluster location
			clusterMarkM4<<<grid_x2, block_x2>>>(cluster_list, cluster_location, cluster_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			int old_cluster_num = cluster_num_;

			exclusiveScan(cluster_location, point_num_ + 1, &cluster_num_);

			int *new_cluster_list;

			checkCudaErrors(cudaMalloc(&new_cluster_list, sizeof(int) * cluster_num_));

			clusterCollectorM4<<<grid_x2, block_x2>>>(cluster_list, new_cluster_list, cluster_location, old_cluster_num);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			// Rebuild matrix
			int *new_sample_matrix;
			int new_sample_size = (cluster_num_ - 1) / BLOCK_SIZE_X + 1;

			checkCudaErrors(cudaMalloc(&new_sample_matrix, sizeof(int) * new_sample_size * new_sample_size));
			checkCudaErrors(cudaMemset(new_sample_matrix, 0, sizeof(int) * new_sample_size * new_sample_size));

			block_size.x = block_x2;
			block_size.y = block_size.z = 1;
			grid_size.x = grid_x2;
			grid_size.y = (old_cluster_num > GRID_SIZE_Y) ? GRID_SIZE_Y : old_cluster_num;
			grid_size.z = 1;

			rebuildSampleMatrixM4<<<grid_size, block_size>>>(sample_matrix, sample_size,
															sub_mat_loc, matrix,
															cluster_list,
															cluster_location,
															new_sample_matrix, new_sample_size);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			int *new_sub_mat_loc;
			int new_sub_mat_num;

			checkCudaErrors(cudaMalloc(&new_sub_mat_loc, sizeof(int) * (new_sample_size * new_sample_size + 1)));
			checkCudaErrors(cudaMemcpy(new_sub_mat_loc, new_sample_matrix, sizeof(int) * new_sample_size * new_sample_size, cudaMemcpyDeviceToDevice));

			exclusiveScan(new_sub_mat_loc, new_sample_size * new_sample_size + 1, &new_sub_mat_num);

			int *new_matrix;

			checkCudaErrors(cudaMalloc(&new_matrix, sizeof(int) * new_sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X));
			checkCudaErrors(cudaMemset(new_matrix, 0, sizeof(int) * new_sub_mat_num * BLOCK_SIZE_X * BLOCK_SIZE_X));

			rebuildAdjacencyMatrixM4<<<grid_size, block_size>>>(sample_matrix, sample_size,
																sub_mat_loc, matrix,
																cluster_list,
																cluster_location,
																new_sub_mat_loc, new_sample_size, new_matrix);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());


			checkCudaErrors(cudaFree(cluster_list));
			cluster_list = new_cluster_list;

			checkCudaErrors(cudaFree(matrix));
			matrix = new_matrix;

			checkCudaErrors(cudaFree(sample_matrix));
			sample_matrix = new_sample_matrix;

			checkCudaErrors(cudaFree(sub_mat_loc));
			sub_mat_loc = new_sub_mat_loc;
		}

		std::cout << "Cluster num = " << cluster_num_ << std::endl;

		itr++;
	} while (hcheck);


	gettimeofday(&end, NULL);

	std::cout << "Iteration = " << timeDiff(start, end) << " number of iterations = " << itr << std::endl;

	renamingClusters(cluster_name_, cluster_location, point_num_);

	checkCudaErrors(cudaMemcpy(cluster_name_host_, cluster_name_, point_num_ * sizeof(int), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(matrix));
	checkCudaErrors(cudaFree(cluster_list));
	checkCudaErrors(cudaFree(cluster_location));
	checkCudaErrors(cudaFree(check));
	checkCudaErrors(cudaFree(sample_matrix));
	checkCudaErrors(cudaFree(sub_mat_loc));
}
