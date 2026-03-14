import tkinter as tk
from tkinter import ttk

#Create the main window
class GUI:
    def __init__(self):
        self.main_window = tk.Tk()

        self.main_window.title("Robot Control Station")
        self.main_window.configure(background="gray")
        
        #Make Coordinate Display Frame
        self.coord_frame = tk.Frame(
            width=225,
            height=200,
            highlightbackground="Black",
            highlightthickness= 2,
            bg = "white"
            )
        self.coord_frame.pack_propagate(False)
        self.coord_frame.pack(anchor="nw")
       
        #Make Coordinate Control Frame
        self.control_coords = tk.Frame(
            width=500,
            height=200,
            highlightbackground="Black",
            highlightthickness= 2,
            bg = "white"
            )
        self.control_coords.pack_propagate(False)
        self.control_coords.pack(side="left")

        self.Position = tk.Label(
            self.coord_frame,
            text="End Effector Position",
            font=("Arial", 12, "bold"),
            bg="white"
            )
        self.Position.pack(anchor="center")

        self.line_bar = tk.Frame(
            self.coord_frame, 
            height=5,          
            width=200,
            bg="black"         
            )
        
        self.separator = ttk.Separator(self.coord_frame, orient='horizontal')
        self.separator.pack(fill='x')

        #Add Widgets To Frames
        self.x_val = add_coordinate_disp(self.coord_frame,"X Coordinate")
        self.y_val = add_coordinate_disp(self.coord_frame,"Y Coordinate")
        self.z_val = add_coordinate_disp(self.coord_frame,"Z Coordinate")
        self.phi_val = add_coordinate_disp(self.coord_frame,"Phi Angle")
        self.Theta_val = add_coordinate_disp(self.coord_frame,"Theta Angle")
        self.psi_val = add_coordinate_disp(self.coord_frame,"Psi Angle")

        self.x_val = add_slider(self.control_coords,"X Control")
        self.y_val = add_slider(self.control_coords,"Y Control")
        self.z_val = add_slider(self.control_coords,"Z Control")
        self.phi_val = add_slider(self.control_coords,"Phi Control")
        self.theta_val = add_slider(self.control_coords,"Theta Control")
        self.psi_val = add_slider(self.control_coords,"Psi Control")

        self.main_window.mainloop()

#Create Coordinate Display function
def add_coordinate_disp(parent, label_text):
        """Creates a row and returns the label that holds the value."""
        row_frame = tk.Frame(parent, bg="white")
        row_frame.pack(fill="x", anchor="center", padx=20,pady=5)

        # The Name Label
        name_label = tk.Label(
            row_frame, text=f"{label_text}:", 
            font=("Arial", 10, "bold"), bg="white", 
            width=15, anchor="e"
        )
        name_label.pack(side="left")

        # The Value Label (The one we actually care about updating)
        value_label = tk.Label(
            row_frame, text="0.00", 
            font=("Arial", 10, "bold"), fg="blue", bg="white"
        )
        value_label.pack(side="left")

        return value_label

#Create Slider Display function
def add_slider(parent,label_text, min_val=-180, max_val=180):
        slider_frame = tk.Frame(parent, bg="white")
        slider_frame.pack(fill="x", anchor="w", padx =20,pady=5)

        name_label = tk.Label(
            slider_frame, text=f"{label_text}:", 
            font=("Arial", 10, "bold"), bg="white", 
            width=15, anchor="e"
        )
        name_label.pack(side="left")

        slider = tk.Scale(
            slider_frame, 
            from_=min_val, 
            to=max_val, 
            orient="horizontal", 
            length=250,        
            bg="white", 
            troughcolor="#D3D3D3",
            activebackground="gray",
            highlightbackground="gray",
            highlightcolor="white",
            highlightthickness=0,
            showvalue=False     
            )
        slider.pack(side="left", padx=20)
       
        return slider

if __name__ == "__main__":
    GUI()











