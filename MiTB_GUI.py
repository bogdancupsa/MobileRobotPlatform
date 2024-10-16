## ------------------------------------------------------------------------------------------------------------------
## - Imports -
import tkinter as tk
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import matplotlib.pyplot as plt
import customtkinter

## ------------------------------------------------------------------------------------------------------------------
## - Inputs/Outputs -
out_data1 =  [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0]
out_data2 =  [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1]

out_data = [out_data1, out_data1]

in_data1 = [1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0]
in_data2 = [0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1]
in_data3 = [1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,1,0,1]
in_data4 = [1,1,1,1,1,1,1,0,0,0,0,1,1,1,0,0,0,1,0,1]

in_data = [in_data1, in_data2, in_data3, in_data4]

# out_data_ts = [10, 30, 40, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 230]

## ------------------------------------------------------------------------------------------------------------------
## - Configs -

ch_config = {
    '0': {
        'name': 0,
        'outputs': 1,
        'inputs': 1,
        'output_voltage': 3,
        'input_voltage' : 3,
    },
    '1' : {
        'name': 1,
        'outputs': 1,
        'inputs': 2,
        'output_voltage': 3,
        'input_voltage' : 3,
    },
    '2' : {
        'name': 2,
        'outputs': 1,
        'inputs': 4,
        'output_voltage': 3,
        'input_voltage' : 3,
    }
}
output_color =  ['black', 'blue']
input_color = ['red', 'orange', 'magenta', 'green', 'cyan', 'purple', 'olive']



## ------------------------------------------------------------------------------------------------------------------
## - Main class -
class MiTB_GUI():
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title('MiTB GUI')

        ## -- Menu Bar --
        self.menubar = tk.Menu(self.root)
        self.filemenu = tk.Menu(self.menubar, tearoff=0)
        self.filemenu.add_command(label='Close', command=exit)
        self.menubar.add_cascade(menu=self.filemenu, label='File')
        self.root.config(menu=self.menubar)

        ## -- Geometry --
        self.root.geometry('300x200')

        ## -- Buttons --
        for btn in range(len(ch_config)):            
            self.btn = customtkinter.CTkButton(self.root, width=100, text=f'Open Chn{btn}', font=('Arial', 12), command=lambda b=ch_config[f'{btn}']['name']: Channel_Page(b)).pack(padx=15, pady=15, expand=True)
## ------------------------------------------------------------------------------------------------------------------
## - Channel page generation class -
class Channel_Page():
    def __init__(self, ch_nr):
        root = tk.Tk()
        root.title(f'Channel {ch_nr} signals')
        root.geometry('800x600')
        root.state('zoomed')                                                                                ## Start window maximized

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Create Widgets --
        main_canvas = customtkinter.CTkScrollableFrame(root, fg_color='transparent')                        ## place the canvas in the main window
        outputs_frame = customtkinter.CTkFrame(main_canvas, fg_color='transparent')                         ## place a frame in the main canvas
        inputs_frame = customtkinter.CTkFrame(main_canvas, fg_color='transparent')                          ## place a frame in the main canvas

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Configure widgets --     

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Pack Widgets --
        main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)                                              ## generate the canvas
        outputs_frame.pack(side=tk.TOP, fill=tk.BOTH, anchor=tk.W, padx=5, pady=5, expand=1)                ## generate the frame
        inputs_frame.pack(side=tk.TOP, fill=tk.BOTH, anchor=tk.W, padx=5, pady=5, expand=1)                 ## generate the frame

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Outputs --
        label_outputs = tk.LabelFrame(outputs_frame, text='Outputs', font=('Arial', 10))                    ## Label frame to encase all output Widgets
        label_outputs.pack(side=tk.TOP, fill=tk.BOTH, anchor=tk.W, padx=5, pady=5)                          ## place it at the top and stretch it all around

        output_bts = []                                                                                     ## list for all buttons to be stored
        output_grphs = []                                                                                   ## list for all graphs to be stored
        for outputs in range(0, ch_config[f'{ch_nr}']['outputs'] * 2, 2):                                   ## create all output Widgets, ch_config[f'{ch_nr}']['outputs'] contains the number of outputs
            btn_out = customtkinter.CTkButton(label_outputs, text='Start signal', width=100, font=('Arial', 12), command=lambda bt_nr=int(outputs/2): plot_output(bt_nr)) ## create the button to start the output
            output_bts.append(btn_out)                                                                      ## add the button to the list

            figure_out = plt.figure(num=f'output{outputs}', figsize=(21, 2.5), dpi=80, layout='constrained')                        ## create the canvas where the signal will be plotted
            ax_out = figure_out.add_subplot()                                                               ## add the axes to the canvas

            grph_out = FigureCanvasTkAgg(figure_out, master=label_outputs)                                  ## create the actual graph canvas, this is where the signal will appear
            toolbar_out = NavigationToolbar2Tk(grph_out, label_outputs, pack_toolbar=False)                 ## add toolbar for each graph

            output_grphs.append((ax_out, grph_out, figure_out))                                             ## add the graph to the list

            btn_out.grid(row=outputs, column=0, padx=5, pady=5, sticky='nw')                                ## button placement inside the grid
            grph_out.get_tk_widget().grid(row=outputs, column=1, padx=5, pady=5, sticky='w')                ## graph placement inside the grid
            toolbar_out.grid(row=outputs+1, column=1, padx=5, pady=5, sticky='nw')                          ## toolbar placement inside grid
            self.axis_config(ax_out, (-1, len(out_data[int(outputs/2)])), (-0.5, ch_config[f'{ch_nr}']['output_voltage'] + 0.5), f'Output {len(output_grphs)}')  ## add labels and etc.

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Inputs --
        label_inputs = tk.LabelFrame(inputs_frame, text='Inputs', font=('Arial', 10))                       ## Label frame to encase all input Widgets
        label_inputs.pack(side=tk.TOP, fill=tk.BOTH, anchor=tk.W, padx=5, pady=5)                           ## place it at the bottom and stretch it all around

        input_grphs = []                                                                                    ## list for all graphs to be stored

        for inputs in range(0, ch_config[f'{ch_nr}']['inputs'] * 2 ,2):                                     ## create all input Widgets, ch_config[f'{ch_nr}']['inputs'] contains the number of inputs
            in_label = tk.Label(label_inputs)                                                               ## create a label to know which input it is    

            figure_in = plt.figure(num=f'input{inputs}', figsize=(21, 2.5), dpi=80, layout='constrained')                         ## create the canvas where the signal will be plotted
            ax_in = figure_in.add_subplot()                                                                 ## add the axes to the canvas
            grph_in = FigureCanvasTkAgg(figure_in, master=label_inputs)                                     ## create the actual graph canvas, this is where the signal will appear
            toolbar_in = NavigationToolbar2Tk(grph_in, label_inputs, pack_toolbar=False)                    ## add toolbar for each graph

            input_grphs.append((ax_in, grph_in, figure_in))                                                 ## add the graph to the list

            in_label.grid(row=inputs, column=0, padx=30, pady=30, sticky='w')                               ## label placement inside the grid
            grph_in.get_tk_widget().grid(row=inputs, column=1, padx=5, pady=5, sticky='w')                  ## graph placement inside the grid
            toolbar_in.grid(row=inputs+1, column=1, padx=5, pady=5, sticky='nw')                            ## toolbar placement inside grid
            in_label.config(text=f'Input {len(input_grphs)}')                                               ## set label name
            self.axis_config(ax_in, (-1, len(in_data[int(inputs/2)])), (-0.5, ch_config[f'{ch_nr}']['input_voltage'] + 0.5), f'Input {len(input_grphs)}')  ## add labels and etc.

        ## ----------------------------------------------------------------------------------------------------------
        ## -- Plot function for Output signals --
        def plot_output(outps):                                                                             ## inps is button number, to know which graph to update based on which input
            output_grphs[outps][0].clear()                                                                  ## clear existing graph
            self.axis_config(output_grphs[outps][0], (-1, len(out_data[outps])), (-0.5, ch_config[f'{ch_nr}']['output_voltage'] + 0.5), f'Output {outps+1}')  ## add labels and etc.
            output_grphs[outps][0].step(range(len(out_data[outps])), [x * 3 for x in out_data[outps]], color=output_color[outps])    ## add the signals to the graph
            output_grphs[outps][1].draw()                                                                   ## draw the input signal
            plot_input()                                                                                    ## also plot the output

        ## ---------------------------------------------------------------------------------------------------------- 
        ## -- Plot function for Input signals --   
        def plot_input():
            for inputs in range(ch_config[f'{ch_nr}']['inputs']):                                           ## plot each ouput based on channel config
                input_grphs[inputs][0].clear()                                                              ## clear existing graph
                self.axis_config(input_grphs[inputs][0], (-1, len(in_data[inputs])), (-0.5, ch_config[f'{ch_nr}']['input_voltage'] + 0.5), f'Input {inputs+1}')  ## add labels and etc.
                input_grphs[inputs][0].step(range(len(in_data[inputs])), [x * 3 for x in in_data[inputs]], color=input_color[inputs])             ## add the signals to the graph
                input_grphs[inputs][1].draw()                                                               ## draw the output signal              


        label_outputs.bind('<Configure>', lambda e: self.figure_resize(e, output_grphs, 'Output'))
        label_inputs.bind('<Configure>', lambda e: self.figure_resize(e, input_grphs, 'Input'))
    ## ---------------------------------------------------------------------------------------------------------- 
    ## -- Resize function for Graphs -- 
    def figure_resize(self, event, grphs, title):
        for graph in range(len(grphs)):
            w, h = event.width, event.height                                                                ## store localy width and height
            event.width = w - 136                                                                           ## substract some padding for current graph
            event.height = 200                                                                              ## add custom height (default pixel size) 
            grphs[graph][1].resize(event)                                                                   ## resize the graph
            self.axis_config(grphs[graph][0], (-1, 20), (-0.5, 3.5), f'{title} {graph+1}')                  ## add labels and etc.
            event.width = w                                                                                 ## restore event width for next graphs
            event.height = h                                                                                ## restore event height for next graphs

    ## ---------------------------------------------------------------------------------------------------------- 
    ## -- Axis configuration function -- axis labels, title, grid, ticks have to be specified everytime the graph is updated
    def axis_config(self, axis, xlimits, ylimits, title):
        axis.autoscale_view(True)
        axis.set_xlabel('Time')                                                                             ## Add label to X axis
        axis.set_ylabel('Volts')                                                                            ## Add label to Y axis
        axis.set_xlim(xlimits)                                                                              ## set default X range
        axis.set_ylim(ylimits)                                                                              ## set default Y range
        axis.grid(visible=1)                                                                                ## add grid lines to the graph
        axis.set_title(title)                                                                               ## set signal title
        axis.set_xticks(range(-1, int(xlimits[1]), 1))                                                      ## set timestamps - will update in the future to support custom TS
        # axis.set_xticks(ts)                                                                                 ## set timestamps - will update in the future to support custom TS

## ------------------------------------------------------------------------------------------------------------------
## - Program start -       
if __name__ == '__main__':
    app = MiTB_GUI()
    app.root.mainloop()


# print([x * 3 for x in in_data1])
# print(range(len(in_data[0])))