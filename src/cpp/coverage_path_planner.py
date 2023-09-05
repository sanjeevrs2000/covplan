from field import Field

def cpp(input_file,width,num_hd=1,theta=0,num_clusters=3,radius=2,utm_=False,utmzone=32):
    
	f = Field(input_file, width, num_hd, theta, utm_,utmzone=utmzone)
	f.headlandGen()
	f.trackGen()
	f.cluster(num_clusters)
	f.tsp_opt()
	f.trajGen(radius)
	f.showField()

