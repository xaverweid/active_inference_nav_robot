#Be aware, the variable "plot_data_IndoorMap_5s" needed for the last plot comes from the "1.1-TaskPerformance_IndoorMap.R" file

library(ggplot2)
library(tidyr)
library(dplyr)
library(extrafont)
library(readxl)

# --- STEP 1: INITIALIZE FONTS (For Arial 12pt) ---
# Run font_import(pattern = "Times", prompt = FALSE) once if you haven't before
loadfonts(device = "pdf", quiet = TRUE)
# --- STEP 2: ASSIGN EXISTING DATA FRAMES ---
# Mapping your pre-imported environment objects
DATA_DIR <- "../data" 

path_HMap_1s <- file.path(DATA_DIR, "h_map/HMap_1s.xlsx")
path_HMap_5s <- file.path(DATA_DIR, "h_map/HMap_5s.xlsx")

df_HMap_1s_raw <- read_xlsx(path_HMap_1s)
df_HMap_5s_raw <- read_xlsx(path_HMap_5s)



# Dynamically force the first column name to "Metric" regardless of its current name
colnames(df_HMap_1s_raw)[1] <- "Metric"
colnames(df_HMap_5s_raw)[1] <- "Metric"


# --- STEP 3: TRANSFORM WIDE TO LONG FORMAT ---
HMap_process_dataset <- function(df, duration_label) {
  df %>%
    # Clean string spacing/capitalization discrepancies if they exist
    mutate(Metric = trimws(tolower(Metric))) %>%
    
    # Filter out only the 5 outcome categories needed for the stacked bars
    # (Adjust strings here if your rows use slightly different naming)
    filter(Metric %in% c("true success", "false success", "collision", "timeout", "other failure")) %>%
    
    # Pivot the 7 algorithm columns into a single long column
    pivot_longer(cols = -Metric, names_to = "Algorithm", values_to = "Count") %>%
    
    # Label the duration group
    mutate(Duration = duration_label)
}

# Run the transformation pipeline
cleaned_HMap_1s <- HMap_process_dataset(df_HMap_1s_raw, "1s Duration")
cleaned_HMap_5s <- HMap_process_dataset(df_HMap_5s_raw, "5s Duration")
plot_data_HMap  <- bind_rows(cleaned_HMap_1s, cleaned_HMap_5s)

# --- STEP 4: STANDARDIZE LABELS & LAYER ORDER (UPDATED) ---
# Capture the exact column order of your algorithms from the original dataset
# (We use [-1] to skip the first column, which is the "Metric" column)
custom_algo_order <- c(
  "Standard AIC", 
  "Deep AIC", 
  "Full-Distribution AIC",
  "Purely Epistemic AIC",
  "D-Optimality",
  "Constrained Random"
)
plot_data_HMap <- plot_data_HMap %>%
  mutate(
    Metric = case_when(
      Metric == "true success"   ~ "True Success",
      Metric == "false success"  ~ "False Success",
      Metric == "collision"      ~ "Collision",
      Metric == "timeout"        ~ "Timeout",
      Metric == "other failure"  ~ "Other Failure",
      TRUE                       ~ Metric
    ),
    # 1. Enforce the exact dataset column order for the X-axis
    Algorithm = factor(Algorithm, levels = custom_algo_order),
    
    # 2. Sets the visual stack ordering from bottom to top
    Metric = factor(Metric, levels = c("Other Failure", "Timeout", "Collision", "False Success", "True Success"))
  )


# --- STEP 5: DEFINE COGNITIVE SCIENCE PALETTE (UPDATED) ---
# High-contrast, easily discriminative academic palette
academic_colors <- c(
  "True Success"       = "#4A6FA5",  # Muted Slate Blue
  "False Success"      = "#D4A373",  # Muted Ochre / Sand
  "Collision" = "#B33939",  # Terracotta / Deep Red
  "Timeout"            = "#2C3E50",  # Dark Charcoal (Highly distinct from grays)
  "Other Failure"      = "#9B86A6"   # Muted Dusty Violet (Highly distinct from everything else)
)

# --- STEP 6: RENDER THE GGPLOT ---
stacked_plot_HMap <- ggplot(plot_data_HMap, aes(x = Algorithm, y = Count, fill = Metric)) +
  geom_bar(stat = "identity", position = "stack", width = 0.7, color = NA) +
  
  # Generates side-by-side panels for 1s vs 5s configurations
  facet_wrap(~Duration, scales = "free_x") +
  scale_fill_manual(values = academic_colors) +
  
  # Assumes 100 trials total per run, setting raw count directly to 100% height
  scale_y_continuous(limits = c(0, 100), breaks = seq(0, 100, 20), expand = c(0, 0)) +
  
  labs(
    x = "Control Algorithm",
    y = "Percentage of Experimental Trials (%)",
    fill = "Trial Outcome"
  ) +
  
  # Strict Academic Theme Customization (Arial, 12pt)
  theme_classic(base_size = 12, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 12),
    axis.title = element_text(face = "bold", size = 12),
    axis.text = element_text(color = "black", size = 11),
    axis.text.x = element_text(angle = 45, hjust = 1), # Cleanly angling algorithm titles
    
    strip.background = element_blank(), 
    strip.text = element_text(face = "bold", size = 12, margin = margin(b = 10)),
    panel.spacing = unit(2, "lines"), 
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 12, margin = margin(b = 8)),
    legend.text = element_text(size = 11),
    
    # Draws the 0.5pt grey bounding box frame with an inner margin cushion
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )

# Output image to plot pane
print(stacked_plot_HMap)


# --- STEP 7: EXPORT HIGH-RESOLUTION VECTOR GRAPH ---
ggsave(
  filename = "1.1_H_Map_Performance_Profiles.png", 
  plot = stacked_plot_HMap, 
  device = "png", 
  width = 11, 
  height = 6, 
  units = "in",
  dpi = 600 # Standard print is 300, 600 is ultra-sharp academic grade
)


# --- STEP 8: FILTER TO 5s DURATION ONLY (both maps) ---
plot_data_HMap_5s <- plot_data_HMap %>%
  filter(Duration == "5s Duration") %>%
  mutate(Map = "H-Map")

plot_data_IndoorMap_5s <- plot_data_IndoorMap_5s %>%
  filter(Duration == "5s Duration") %>%
  mutate(Map = "Indoor Map")

# --- STEP 9: COMBINE INTO A SINGLE DATAFRAME ---
plot_data_combined_5s <- bind_rows(plot_data_HMap_5s, plot_data_IndoorMap_5s) %>%
  mutate(Map = factor(Map, levels = c("Indoor Map", "H-Map")))

# --- STEP 10: RENDER THE COMBINED GGPLOT ---
stacked_plot_combined_5s <- ggplot(plot_data_combined_5s, aes(x = Algorithm, y = Count, fill = Metric)) +
  geom_bar(stat = "identity", position = "stack", width = 0.7, color = NA) +
  
  # Facet by Map so each environment gets its own panel, but shares axes/legend
  facet_grid(~ Map, scales = "free_x", space = "free_x") +
  
  scale_fill_manual(values = academic_colors) +
  
  scale_y_continuous(limits = c(0, 100), breaks = seq(0, 100, 20), expand = c(0, 0)) +
  
  labs(
    x = "Control Algorithm",
    y = "Percentage of Experimental Trials (%)",
    fill = "Trial Outcome"
  ) +
  
  theme_classic(base_size = 11, base_family = "Arial") +
  theme(
    text = element_text(family = "Arial", size = 11),
    axis.title = element_text(face = "bold", size = 11),
    axis.text = element_text(color = "black", size = 10),
    axis.text.x = element_text(angle = 45, hjust = 1),
    
    strip.background = element_blank(),
    strip.text = element_text(face = "bold", size = 11, margin = margin(b = 10)),
    panel.spacing = unit(2, "lines"),
    
    legend.position = "right",
    legend.direction = "vertical",
    legend.title = element_text(face = "bold", size = 11, margin = margin(b = 8)),
    legend.text = element_text(size = 10),
    
    legend.background = element_rect(color = "grey85", fill = "white", linewidth = 0.5),
    legend.margin = margin(t = 10, r = 10, b = 10, l = 10)
  )
# Output image to plot pane
print(stacked_plot_combined_5s)
# --- STEP 11: EXPORT HIGH-RESOLUTION VECTOR GRAPH (COMBINED) ---
ggsave(
  filename = "1.1_Combined_Performance_Profiles_5s.png", 
  plot = stacked_plot_combined_5s, 
  device = "png", 
  width = 11, 
  height = 6, 
  units = "in",
  dpi = 600
)