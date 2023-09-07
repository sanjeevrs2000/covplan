from field import Field

def cpp(input_file,width,num_hd=1,theta=0,num_clusters=3,radius=2):
    
	f = Field(input_file, width, num_hd, theta)
	f.headlandGen()
	f.trackGen()
	f.cluster(num_clusters)
	f.tsp_opt()
	f.trajGen(radius)
	f.showField()

