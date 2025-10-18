"""
Sphinx extension to auto-document YAML parameter files.
"""

import yaml
from pathlib import Path
from docutils import nodes
from docutils.statemachine import ViewList
from sphinx.util.docutils import SphinxDirective
from sphinx.util.nodes import nested_parse_with_titles


class YAMLParamsDirective(SphinxDirective):
    """
    Directive to automatically document ROS 2 YAML parameter files.
    
    Usage::
    
        .. yaml-params:: ../../config/params.yaml
           :node: kitti_publisher_node
    """
    
    required_arguments = 1  # Path to YAML file
    option_spec = {
        'node': str,  # Optional: specific node to document
    }
    
    def run(self):
        # Get the YAML file path relative to the source directory
        env = self.state.document.settings.env
        yaml_path = Path(env.srcdir) / self.arguments[0]
        
        if not yaml_path.exists():
            error = self.state_machine.reporter.warning(
                f'YAML file not found: {yaml_path}',
                line=self.lineno
            )
            return [error]
        
        # Read and parse YAML
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Generate RST content
        rst_content = self._generate_rst(data, yaml_path)
        
        # Parse the generated RST
        node = nodes.section()
        node.document = self.state.document
        nested_parse_with_titles(self.state, ViewList(rst_content.split('\n')), node)
        
        return node.children
    
    def _generate_rst(self, data, yaml_path):
        """Generate reStructuredText from YAML data."""
        rst_lines = []
        
        # Add file reference
        rst_lines.append(f"**File:** ``{yaml_path.name}``")
        rst_lines.append("")
        
        # Add literal include of the file
        rst_lines.append(".. code-block:: yaml")
        rst_lines.append("   :caption: Full configuration file")
        rst_lines.append("   :linenos:")
        rst_lines.append("")
        
        with open(yaml_path, 'r') as f:
            for line in f:
                rst_lines.append(f"   {line.rstrip()}")
        
        rst_lines.append("")
        rst_lines.append("Parameters")
        rst_lines.append("----------")
        rst_lines.append("")
        
        # Get node name from options or use first key
        node_name = self.options.get('node')
        
        if node_name:
            params = data.get(node_name, {}).get('ros__parameters', {})
        else:
            # Use first node found
            first_key = next(iter(data.keys()))
            params = data[first_key].get('ros__parameters', {})
            node_name = first_key
        
        rst_lines.append(f"Node: ``{node_name}``")
        rst_lines.append("")
        
        # Generate parameter table
        if params:
            rst_lines.extend(self._generate_param_table(params))
        
        return '\n'.join(rst_lines)
    
    def _generate_param_table(self, params):
        """Generate a table documenting parameters."""
        lines = [
            ".. list-table::",
            "   :header-rows: 1",
            "   :widths: 25 15 60",
            "",
            "   * - Parameter",
            "     - Default Value",
            "     - Type",
        ]
        
        for param_name, param_value in params.items():
            # Determine type
            param_type = type(param_value).__name__
            
            # Format value for display
            if isinstance(param_value, str):
                display_value = f"``'{param_value}'``"
            else:
                display_value = f"``{param_value}``"
            
            lines.extend([
                f"   * - ``{param_name}``",
                f"     - {display_value}",
                f"     - {param_type}",
            ])
        
        return lines


def setup(app):
    """Setup function to register the extension."""
    app.add_directive('yaml-params', YAMLParamsDirective)
    
    return {
        'version': '0.1',
        'parallel_read_safe': True,
        'parallel_write_safe': True,
    }