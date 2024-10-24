# Radar 
Radar works using the transmission and detection of electromagnetic waves as seen in the following image:

<img width="600" alt="image" src="https://github.com/user-attachments/assets/be1eade4-a183-4a43-847d-6b2383d6795b">

The frequency of electromagnetic energy used for radar is unaffected by darkness and penetrates fog and clouds. This permits radar systems to determine the position of road targets that are invisible to the naked eye because of distance, darkness, or weather. The automotive radar generally operates at W band (76GHz - 81GHz). The signal at this frequency is referred to as millimeterWave since the wavelength is in mm.

## FMCW
FMCW radar (Frequency-Modulated Continuous Wave radar) is a specialized radar sensor that emits continuous transmission power. Its capability to measure very small distances to a target, along with its ability to simultaneously determine both the target's range and relative velocity, makes it the preferred radar type for automotive applications.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/40188577-1159-431c-b89b-2a7a8436853e">

A Frequency Modulated Continuous Wave (FMCW) signal is characterized by a frequency that changes over time, either increasing or decreasing. These variations are commonly referred to as upramps and downramps. The two most common waveform patterns used in FMCW radars are sawtooth and triangular. The sawtooth waveform typically employs only the upramps, while the triangular waveform utilizes both upramps and downramps.

Each chirp is defined by its slope. The slope is given by its chirp frequency bandwidth B or Bsweep (y-axis) and its chirp time Ts (x-axis). Hence, Slope=B/Ts

### FMCW Hardware Overview

<img width="600" alt="image" src="https://github.com/user-attachments/assets/e48000ab-36a5-4f58-8a21-7d459c7f1227">

**Frequency Synthesizer:** The frequency synthesizer is the component that generates the frequency to bring the chirp frequency all the way to 77GHz in case of automotive radar.

**Power Amp:** The power amp amplifies the signal so the signal can reach long distances. Since the signal attenuates as it radiates, it needs higher power (amplitude) to reach targets at greater distances.

**Antenna:** The antenna converts the electrical energy into electromagnetic waves which radiate through the air, hit the target, and get reflected toward the radar receiver antenna. The Antenna also increases the strength of the signal by focusing the energy on the desired direction. Additionally, the antenna pattern determines the field of view for the radar.

**Mixer:** In FMCW radar, the mixer multiplies the return signal with the sweeping signal generated by the frequency synthesizer. The operation works as frequency subtraction to give the frequency delta - also known as frequency shift or Intermediate frequency (IF). IF = Synthesizer Frequency - Return Signal Frequency.

**Processor:** The processor is the processing unit where all the Digital Signal processing, Detection, Tracking, Clustering, and other algorithms take place. This unit could be a microcontroller or even an FPGA.

## Radar Cross Section
The size and ability of a target to reflect radar energy is defined by a single term, σ, known as the radar cross-section, which has units of m^2. This unit shows that the radar cross section is an area. The target radar cross sectional area depends on:

•	The target’s physical geometry and exterior features:

Smooth edges or surface would scatter the waves in all directions, hence lower RCS. Whereas sharp corners will focus the return signal back in the direction of the source leading to higher RCS. (Image below for different target geometries)

•	The direction of the illuminating radar,

•	The radar transmitter’s frequency,

•	The material used in cars, trucks, bicycles, and even in some cases, the clothing material for pedestrians.

<img width="600" alt="image" src="https://github.com/user-attachments/assets/0ade1b81-a3ed-4694-a464-dd0a9cf41436">

Returns from Different Target Geometries:

<img width="300" alt="image" src="https://github.com/user-attachments/assets/22ef9bf1-7b4f-4ea2-b7c4-ff1df0581f8d">

### RCS Units
This RCS can also be defined using a logarithmic value (dB), since it increases the return signal strength. The formula for converting from RCS to dB is given by: 
<img width="200" alt="image" src="https://github.com/user-attachments/assets/09f3ca7a-2010-429d-a45f-f49ab78dff98">

The following table shows RCS values for different targets in both m^2 and dB. You can use the formula above to see how the two columns of the table are related. For example, from the table below we can see that for an automobile: 
<img width="200" alt="image" src="https://github.com/user-attachments/assets/5d78023f-2503-4ea6-8be5-6ed45e907d8d">


<img width="200" alt="image" src="https://github.com/user-attachments/assets/fdb67f2f-e240-4036-a478-294d85cc0a97">

## Range Equation Overview
Using the Radar Range equation, we can design the radar transmitter, receiver, and antenna to have the desired power, gain and noise performance to meet the range requirements.

A long-range radar designed to cover 300m range and detect a target with smaller cross section would need higher transmit power and more antenna gain as compared to a short range radar designed to cover just 50m for similar target. A target with higher cross section can be detected at a longer range as compared to a target with smaller cross section.

<img width="200" alt="image" src="https://github.com/user-attachments/assets/015ff08a-251b-4802-8c74-94e5273e0d27">

•	**R** - Maximum Range a radar can detect targets.

•	**Ps** - Transmitted Power from Radar (dBm)

•	**G - Gain of the Transmit/Receive Antenna (dBi)

•	**λ** - Wavelength of the signal (m)

•	**σ** - radar cross section (m2)

•	**PE** - Minimum received power radar can detect.

## Range Estimation

<img width="600" alt="image" src="https://github.com/user-attachments/assets/ad3bca63-7f31-4bf5-9ec4-991171e7e533">

Radar determines the range of the target by measuring the trip time of the electromagnetic signal it radiates. It is known that EM wave travels at a known speed (300,000,000 m/s), so to determine the range the radar needs to calculate the trip time, by measuring the shift in the frequency.

<img width="600" alt="image" src="https://github.com/user-attachments/assets/ecd671b6-1482-4e2b-83d5-edb7984f0cd1">

## Doppler Estimation

<img width="400" alt="image" src="https://github.com/user-attachments/assets/84e80acc-42a4-4681-ad03-1f53aa1cd624">

As per doppler theory an approaching target will shift an emitted and reflected frequency higher, whereas a receding target will shift both frequencies to be lower than the transmitted frequency.

### FMCW Doppler Measurements

<img width="600" alt="image" src="https://github.com/user-attachments/assets/9f12ab8e-e4bf-4113-be3d-962f856b96b1">

There will be a shift in the received signal frequency due to the doppler effect of the target’s velocity. The doppler shift is directly proportional to the velocity of the target as shown below.

<img width="150" alt="image" src="https://github.com/user-attachments/assets/40cba6a1-8cb0-4fa7-a0e8-af9572e01081">

•	**fD:** shift in the transmitted frequency due to the doppler

•	**νr:** relative velocity of the target

•	**λ:** wavelength of the signal

By measuring the shift in the frequency due to doppler, radar can determine the velocity. The receding target will have a negative velocity due to the frequency dropping lower, whereas the approaching target will have positive velocity as the frequency shifts higher.

### Doppler Phase Shift

We calculate the doppler frequency by measuring the rate of change of phase. The phase change occurs due to small displacement of a moving target for every chirp duration. Since each chirp duration is generally in microseconds, it results in small displacement in mm (millimeters). These small displacements for every chirp lead to change in phase. Using this rate of change of phase we can determine the doppler frequency.

If the path between a target and the radar is changed by an amount Δx, the phase of the wave received by radar is shifted by

<img width="300" alt="image" src="https://github.com/user-attachments/assets/06e14cd9-6033-4d1b-8d15-e6ebec4f493b">

where λ and f are, respectively, the wavelength and frequency of the signal and c is the speed of propagation. The resulting change in observed frequency is

<img width="150" alt="image" src="https://github.com/user-attachments/assets/ec0ea143-b8a8-4682-b380-a59cc2ad2f08">

Where Δt is the time taken for the observation of the phase change.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/3b1a480f-c7a9-4d43-ab01-4f02c7f31d2e">

## Fast Fourier Transform (FFT)
For a radar to efficiently process these measurements digitally, the signal needs to be converted from analog to digital domain and further from time domain to frequency domain to do the spectral analysis of the signal and determine the shifts in frequency due to range and doppler.

ADC (Analog Digital Converter) converts the analog signal into digital, then the Fast Fourier Transform is used to convert the signal from time domain to frequency domain. 

As seen in the image below, the Range FFTs are applied on every sample on each chirp. Since each chirp is sampled N times, it will generate a range FFT block of N * (Number of chirps). These FFT blocks are also called FFT bins.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/aae963cf-230a-4142-816b-41af2454f16a">

## 2D FFT

Once the range bins are determined by running range FFT across all the chirps, a second FFT is implemented along the second dimension to determine the doppler frequency shift. As discussed, the doppler is estimated by processing the rate of change of phase across multiple chirps. Hence, the doppler FFT is implemented after all the chirps in the segment are sent and range FFTs are run on them.

The output of the first FFT gives the beat frequency, amplitude, and phase for each target. This phase varies as we move from one chirp to another (one bin to another on each row) due to the target’s small displacements. Once the second FFT is implemented it determines the rate of change of phase, which is nothing but the doppler frequency shift.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/d259e20d-6ca4-433d-8ce8-e944e9851490">

After 2D FFT each bin in every column of block represents increasing range value and each bin in the row corresponds to a velocity value.

The output of Range Doppler response represents an image with Range on one axis and Doppler on the other. This image is called Range Doppler Map (RDM). These maps are often used as user interface to understand the perception of the targets.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/5a09ed38-1ac7-454f-bce1-cf5564764543">

Radar not only receives the reflected signals from the objects of interest, but also from the environment and unwanted objects. The backscatter from these unwanted sources is called as clutter.

One technique to remove clutter is to remove the signals having 0 doppler velocity. Since, the clutter in the driving scenario are often created by the stationary targets, the 0 doppler filtering can help get rid of them.
The downside of 0 doppler filtering is that the radar would not be able to detect the stationary targets in its path. This would lead to detection failures.

**fixed clutter thresholding:**

Another technique is to use fixed clutter thresholding. With fixed thresholding, signal below the threshold value is rejected. With this method, if the detection threshold is set too high, there will be very few false alarms, but it will also mask the valid targets. If the threshold is set too low, then it would lead to too many false alarms. In other words, the false alarm rate would be too high.

The false alarm rate is the rate of erroneous radar detections by noise or other interfering signals. It is a measure of the presence of detected radar targets when there is no valid target present.

**Dynamic Thresholding:**
Another approach to clutter thresholding is to use dynamic thresholding. Dynamic thresholding involves varying the threshold level to reduce the false alarm rate.

## CFAR
The false alarm issue can be resolved by implementing the constant false alarm rate. CFAR varies the detection threshold based on the vehicle surroundings. The CFAR technique estimates the level of interference in radar range and doppler cells “Training Cells” on either or both the side of the “Cell Under Test”. The estimate is then used to decide if the target is in the Cell Under Test (CUT).

<img width="400" alt="image" src="https://github.com/user-attachments/assets/7bf7630f-ef73-485e-bde1-02fdbdcd4942">

## Clustering
The algorithm is based on the Euclidean distance, it groups the detection points based on their proximity measured by the Euclidean distance between those points.

All the detection points that are within the size of the target are considered as one cluster, merged into a centroid position. Each cluster is now assigned a new range and velocity, which is the mean of measured range and velocity of all the detection points that form the cluster. This allows valid tracking for each target.

