import os
import re

# Update components.html (Slide 7)
path_components = '/home/ubuntu/Inverter-Design/presentation_v2/components.html'
with open(path_components, 'r') as f:
    html = f.read()

# Replace the text to include the new reasoning
html = html.replace(
    '<li><strong>Switch:</strong> Wolfspeed CAB450M12XM3 (1200V, 450A SiC module)</li>',
    '<li><strong>Switch:</strong> Wolfspeed CAB450M12XM3 (1200V, 450A SiC module)<br><span style="font-size: 14px;">- 1200V provides conservative 33% utilization, within IPC-9592B 80% derating limit<br>- Positive temperature coefficient (PTC) of Rds_on prevents thermal runaway</span></li>'
)

html = html.replace(
    '<li><strong>Capacitor:</strong> 680 µF Film Capacitor</li>',
    '<li><strong>Capacitor:</strong> 680 µF Film Capacitor<br><span style="font-size: 14px;">- Selected over electrolytic for ultra-low ESR, high ripple tolerance, and 100k+ hr lifetime</span></li>'
)

with open(path_components, 'w') as f:
    f.write(html)


# Update fc_vs_npc.html (Slide 5)
path_fc = '/home/ubuntu/Inverter-Design/presentation_v2/fc_vs_npc.html'
with open(path_fc, 'r') as f:
    html = f.read()

html = html.replace(
    '<li><strong>Dynamic Balancing:</strong> Redundant switching states naturally balance the flying capacitor regardless of load power factor.</li>',
    '<li><strong>Dynamic Balancing:</strong> Redundant switching states under PD-PWM naturally balance the flying capacitor regardless of load power factor (validated mechanism).</li>'
)

with open(path_fc, 'w') as f:
    f.write(html)

# Update switching_losses.html (Slide 9)
path_losses = '/home/ubuntu/Inverter-Design/presentation_v2/switching_losses.html'
with open(path_losses, 'r') as f:
    html = f.read()

html = html.replace(
    '<li><strong>Analytical Model:</strong> Uses the standard Graovac-Purschel method based on frequency modulation ratio (mf).</li>',
    '<li><strong>Analytical Model:</strong> Uses the standard Graovac-Purschel method.<br><span style="font-size: 14px;">- Topology-dependent, device-agnostic math applies perfectly to SiC MOSFETs<br>- Uses resistive conduction (I²R) and linear switching energy scaling</span></li>'
)

with open(path_losses, 'w') as f:
    f.write(html)

print("Updated presentation slides.")
