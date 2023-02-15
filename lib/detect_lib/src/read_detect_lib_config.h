#define SetYAMLAttribute(doc, cfg, attr_name) {doc["" #attr_name ""] >> cfg->attr_name;}

DetectLibConfig* yaml_to_config(char *yaml_file)
{
  std::ifstream fh(yaml_file);
  YAML::Parser parser(fh);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  DetectLibConfig *cfg = new DetectLibConfig;
  SetYAMLAttribute(doc, cfg, model_path)
  SetYAMLAttribute(doc, cfg, input_width)
  SetYAMLAttribute(doc, cfg, input_height)
  SetYAMLAttribute(doc, cfg, num_output_boxes)
  SetYAMLAttribute(doc, cfg, confidence_threshold)
  SetYAMLAttribute(doc, cfg, threads)
  SetYAMLAttribute(doc, cfg, subsample_major)
  SetYAMLAttribute(doc, cfg, subsample_minor)
  SetYAMLAttribute(doc, cfg, enable_gpu)
  SetYAMLAttribute(doc, cfg, exact_subsample)
  SetYAMLAttribute(doc, cfg, num_classes)
  SetYAMLAttribute(doc, cfg, output_scale)
  SetYAMLAttribute(doc, cfg, input_channels)
  SetYAMLAttribute(doc, cfg, skip_input_channel)
  fh.close();
  return cfg;
}