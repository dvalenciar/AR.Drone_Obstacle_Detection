import cv2
import numpy as np

'''
This code make a draw a grid into the image
I made this because i need to see in which regions are objects
'''


def draw_regions(img, Nx=100, Ny=100):

	#este for es solo para dibujar los cuadrados 
	for i in range(0,img.shape[0]+Nx, Nx):
		for j in range(0,img.shape[1]+Ny, Ny):
			cv2.rectangle(img,(j,i),(j+Ny,Ny),(0,255,0),1)	
				

def divide_regions(img,Nx=100, Ny=100):

	celdas = []

	for i in range(0,img.shape[0], Nx):

		  for j in range(0,img.shape[1], Ny):

			block = img[i:i+Nx,j:j+Ny]
			celdas.append(block)

			#cv2.imshow("Block ", block)
			#cv2.waitKey(0)
			
	return celdas





if __name__ == "__main__":
	

	img = cv2.imread  ('DroneCamara.png')

	draw_regions(img)
	regions = divide_regions(img) 
	


	cv2.imshow("Imagen Frame ", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	
