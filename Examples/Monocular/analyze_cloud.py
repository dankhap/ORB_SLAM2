import numpy as np
import random
import matplotlib.pyplot as plt
import open3d as o3d
from sklearn.decomposition import PCA
import scipy.spatial as sp
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def draw_rect(ax, verts, c='black'):
    codes = [Path.MOVETO] + [Path.LINETO]*(len(verts) - 2) + [Path.CLOSEPOLY]
    verts = verts + [verts[0]]
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='none', edgecolor=c, lw=2)
    ax.add_patch(patch) 
    ax.axis('equal')

def get_rotating_caliper_bbox_list(hull_points_2d):
    """
    hull_points_2d: array of hull points. each element should have [x,y] format
    """
    # Compute edges (x2-x1,y2-y1)
    edges = np.zeros( (len(hull_points_2d)-1,2) ) # empty 2 column array
    for i in range( len(edges) ):
        edge_x = hull_points_2d[i+1,0] - hull_points_2d[i,0]
        edge_y = hull_points_2d[i+1,1] - hull_points_2d[i,1]
        edges[i] = [edge_x,edge_y]
    # Calculate edge angles   atan2(y/x)
    edge_angles = np.zeros( (len(edges)) ) # empty 1 column array
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.arctan2( edges[i,1], edges[i,0] )
    # Check for angles in 1st quadrant
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.abs( edge_angles[i] % (np.pi/2) ) # want strictly positive answers
    #print "Edge angles in 1st Quadrant: \n", edge_angles
    # Remove duplicate angles
    edge_angles = np.unique(edge_angles)
    #print "Unique edge angles: \n", edge_angles
    bbox_list=[]
    for i in range( len(edge_angles) ):
        # Create rotation matrix to shift points to baseline
        # R = [ cos(theta)      , cos(theta-PI/2)
        #       cos(theta+PI/2) , cos(theta)     ]
        R = np.array([ [ np.cos(edge_angles[i]), np.cos(edge_angles[i]-(np.pi/2)) ], [ np.cos(edge_angles[i]+(np.pi/2)), np.cos(edge_angles[i]) ] ])
        # Apply this rotation to convex hull points
        rot_points = np.dot(R, np.transpose(hull_points_2d) ) # 2x2 * 2xn
        # Find min/max x,y points
        min_x = np.nanmin(rot_points[0], axis=0)
        max_x = np.nanmax(rot_points[0], axis=0)
        min_y = np.nanmin(rot_points[1], axis=0)
        max_y = np.nanmax(rot_points[1], axis=0)
        # Calculate height/width/area of this bounding rectangle
        width = max_x - min_x
        height = max_y - min_y
        area = width*height
        # Calculate center point and restore to original coordinate system
        center_x = (min_x + max_x)/2
        center_y = (min_y + max_y)/2
        center_point = np.dot( [ center_x, center_y ], R )
        # Calculate corner points and restore to original coordinate system
        corner_points = np.zeros( (4,2) ) # empty 2 column array
        corner_points[0] = np.dot( [ max_x, min_y ], R )
        corner_points[1] = np.dot( [ min_x, min_y ], R )
        corner_points[2] = np.dot( [ min_x, max_y ], R )
        corner_points[3] = np.dot( [ max_x, max_y ], R )
        bbox_info = [edge_angles[i], area, width, height, min_x, max_x, min_y, max_y, corner_points, center_point]
        bbox_list.append(bbox_info)
    return bbox_list

def main():
    rows = []
    with open('CSC2134.txt', 'r') as f:
        lines = f.readlines()
        for l in lines:
            coords = l.split()
            if len(coords) == 3:
                x, y, z = coords
                rows.append([float(x), float(y), float(z)])

    print(len(rows))
    cloud = np.array(rows)
    points_num = cloud.shape[0]
    ransac_itr = 10
    score = 0
    for i in range(ransac_itr):
        subindex = random.sample(range(points_num), int(points_num*0.01))
        print(f"points selected: {len(subindex)}")
        cloud_subset = cloud[subindex][:, [0,2]]
        hull = sp.ConvexHull(cloud_subset)
        hull_xy = []
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for simplex in hull.simplices:
            plt.plot(cloud_subset[simplex, 0], cloud_subset[simplex, 1], 'k-')
        for simplex in hull.vertices:
            hull_xy.append(cloud_subset[simplex])
        hull_xy = np.array(hull_xy)
        bbox_list = get_rotating_caliper_bbox_list(hull_points_2d=hull_xy)
        min_box = min(bbox_list, key= lambda t: t[1])
        rect = np.array(min_box[8])
        ax.plot(rect[[0,1],0], rect[[0,1],1], color='red')
        ax.plot(rect[[1,2],0], rect[[1,2],1], color='blue')
        ax.plot(rect[[2,3],0], rect[[2,3],1], color='green')
        ax.plot(rect[[3,0],0], rect[[3,0],1], color='black')
        eps = find_epsilon(rect, cloud_subset)
        rect = scale_rect(rect,eps)
        score = evaluate_rect(rect)
        if score > max_score:
            max_score = score
            best_rect = rect
        ax.scatter(cloud_subset[:,0], cloud_subset[:,1])
        plt.show()
        # plt.plot(cloud_subset[simplex, 0], cloud_subset[simplex, 2], 'k-')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    draw_rect(ax, min_box[8]) #corner_points)

    # plt.scatter(cloud_subset[:,0], cloud_subset[:,2])
    plt.show()
    # pca = PCA(n_components=1)
    # flat = pca.fit_transform(cloud)
    # # flat = flat[(flat[:,0]<0.2) & (flat[:,1] < 0.2)]
    # # plt.scatter(flat[:,0], flat[:,1])
    # plt.scatter(flat[:,0], np.zeros(flat.shape[0]))
    # plt.show()
    # orig = pca.inverse_transform(flat)
    # print(orig.shape)
    # plt.scatter(orig[:,0], orig[:,2])
    # plt.show()
    # # pcd = o3d.geometry.pointcloud()
    # pcd.points = o3d.utility.vector3dvector(cloud)
    # o3d.visualization.draw_geometries([pcd])
def load_pcd(path):

    read_coords = 0
    rows = []
    with open(path, 'r') as f:
        lines = f.readlines()
        for l in lines:
            coords = l.split()
            if read_coords == 1:
                read_coords += 1
            if coords[0].startswith("DATA"):
                read_coords += 1
            if len(coords) == 3 and read_coords > 1:
                x, y, _ = coords
                rows.append([float(x), float(y)])
                print(f"added: {coords}")
    points = np.array(rows)
    return points

def visualize_points():
    # points = load_pcd('cleaned_original.pcd')
    # print("loaded cloud, loading min")
    # min_rect = load_pcd('min_eps_model.pcd') 
    # print("loading avg eps")
    # avg_rect = load_pcd('avg_eps_model.pcd') 
    # print()
    # ma_rect = load_pcd('ma_eps_model.pcd') 
    subcloud = load_pcd('subsample.pcd')
    cvhull = load_pcd('cvhull.pcd')
    brect = load_pcd('rect_before_scale.pcd')
    arect = load_pcd('rect_afterscale.pcd')
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(subcloud[:,0], subcloud[:,1])
    # draw_rect(ax, min_rect, 'red')
    # draw_rect(ax, avg_rect, 'green')
    # draw_rect(ax, ma_rect, 'blue')
    draw_rect(ax, cvhull, 'black')
    draw_rect(ax, brect, 'red')
    draw_rect(ax, arect, 'blue')

    plt.show()
if __name__ == "__main__":
    # main()
    visualize_points()
